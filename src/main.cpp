#include "glew.h"
#include "freeglut.h"
#include "glm.hpp"
#include "ext.hpp"
#include <iostream>
#include <cmath>
#include <vector>

#include "Shader_Loader.h"
#include "Render_Utils.h"
#include "Camera.h"
#include "Texture.h"
#include "Physics.h"

using namespace std;

// scene properties
namespace Objects {
    struct Properties {
        glm::vec3 size, pos;
    };

    const int numPins = 10;
    const float offset = 21;
    Properties  ball{ { 0.7, 0.7, 0.7 },{ -30, 0.25, 0 } },
        ground{ { 6, 6, 6 },{ 10, 0.25, 0 } },
        pins[numPins] = {
                { { 0.3, 0.3, 0.3 },{  3 + offset, 0.25, 0 } },
                { { 0.3, 0.3, 0.3 },{  3.5 + offset, 0.25, -0.25 } },
                { { 0.3, 0.3, 0.3 },{  3.5 + offset, 0.25, 0.25 } },
                { { 0.3, 0.3, 0.3 },{  4.0 + offset, 0.25, 0 } },
                { { 0.3, 0.3, 0.3 },{  4.0 + offset, 0.25, -0.5 } },
                { { 0.3, 0.3, 0.3 },{  4.0 + offset, 0.25, 0.5 } },
                { { 0.3, 0.3, 0.3 },{  4.5 + offset, 0.25, -0.25 } },
                { { 0.3, 0.3, 0.3 },{  4.5 + offset, 0.25, -0.75 } },
                { { 0.3, 0.3, 0.3 },{  4.5 + offset, 0.25, 0.25 } },
                { { 0.3, 0.3, 0.3 },{  4.5 + offset, 0.25, 0.75 } }
    };
}

Core::Shader_Loader shaderLoader;
GLuint programColor;
GLuint programTexture;

obj::Model planeModel, sphereModel, pinModel;
PxVec3 vertexes[1647];
GLuint objectTexture, groundTexture, pinTexture;
PxVec3 firstPinPos;
glm::vec3 cameraPos = glm::vec3(-40, 2.5, 0);
glm::vec3 cameraDir;
glm::vec3 cameraSide;
float cameraAngle = 1.58f;
float camX = cameraPos.x;
float camZ = cameraPos.z;
float lastPosition = 0;
//telling camera to look at specific object

glm::mat4 cameraMatrix, perspectiveMatrix;

glm::vec3 lightDir = glm::normalize(glm::vec3(0.5, -1, -0.5));

int score = 0;
// Initalization of physical scene (PhysX)
Physics pxScene(9.8 /* gravity (m/s^2) */);

// fixed timestep for stable and deterministic simulation
const double physicsStepTime = 1.f / 60.f;
double physicsTimeToProcess = 0;
glm::mat4 view;
vector<double> startingPositions;
// physical objects
PxMaterial* material = nullptr;
PxMaterial* ballMaterial = nullptr;
PxRigidStatic* bodyGround = nullptr;
PxRigidDynamic* bodyHandle = nullptr,
* pinHandle = nullptr,
* bodyPins[Objects::numPins] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };

// renderable objects (description of a single renderable instance)
struct Renderable {
    obj::Model* model;
    glm::mat4 localTransform, physicsTransform;
    GLuint textureId;
};
Renderable rendGround, rendHandle, rendPin, rendPins[10];
vector<Renderable*> renderables;
vector<int> pinsDownIndexes;

void initRenderables()
{
    //get starting pin positions x'es
    for (int i = 0; i < 10; i++) {
        startingPositions.push_back(Objects::pins[i].pos.x);
    }
    // load models
    planeModel = obj::loadModelFromFile("models/wenju.obj");
    sphereModel = obj::loadModelFromFile("models/sphere.obj");
    pinModel = obj::loadModelFromFile("models/bowlingPin.obj");
    int j = 0;
    for (int i = 0; i < pinModel.vertex.size(); i += 3) {
        vertexes[j] = PxVec3(pinModel.vertex[i] * 0.3, pinModel.vertex[i + 1] * 0.3, pinModel.vertex[i + 2] * 0.3);
        j++;
    }
    // load textures
    groundTexture = Core::LoadTexture("textures/bowling_lane.bmp");
    objectTexture = Core::LoadTexture("textures/red.jpg");
    pinTexture = Core::LoadTexture("textures/pinTexture.jpg");

    // This time we organize all the renderables in a list
    // of basic properties (model, transform, texture),
    // to unify their rendering and simplify their managament
    // in connection to the physics simulation

    // create ground
    rendGround.model = &planeModel;
    rendGround.textureId = groundTexture;
    rendGround.localTransform = glm::rotate(29.845f, glm::vec3(0.f, 0.f, 1.f)) * glm::rotate(29.843f, glm::vec3(0.f, 1.f, 0.f)) * glm::scale(Objects::ground.size * 0.4f);
    renderables.emplace_back(&rendGround);

    // create handle
    rendHandle.model = &sphereModel;
    rendHandle.textureId = objectTexture;
    rendHandle.localTransform = glm::scale(Objects::ball.size * 0.5f);
    renderables.emplace_back(&rendHandle);

    //create Pin
    for (int i = 0; i < Objects::numPins; i++) {
        rendPins[i].model = &pinModel;
        rendPins[i].textureId = pinTexture;
        rendPins[i].localTransform = glm::scale(Objects::pins[i].size);
        renderables.emplace_back(&rendPins[i]);
    }

}


PxBoxGeometry sizeToPxBoxGeometry(glm::vec3 const& size) {
    auto h = size * 0.5f;
    return PxBoxGeometry(h.x, h.y, h.z);
}

vector<PxRigidDynamic*> pinsBody;
void createDynamicPin(PxRigidDynamic*& body, Renderable* rend, glm::vec3 const& pos, glm::vec3 const& size)
{
    PxConvexMeshDesc convexDesc;
    convexDesc.points.count = 1647;
    convexDesc.points.stride = sizeof(PxVec3);
    convexDesc.points.data = vertexes;
    convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

    PxDefaultMemoryOutputStream buf;
    PxConvexMeshCookingResult::Enum result;
    if (!pxScene.cooking->cookConvexMesh(convexDesc, buf, &result))
        cout << "can't initialize mesh";

    PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
    PxConvexMesh* convexMesh = pxScene.physics->createConvexMesh(input);

    body = pxScene.physics->createRigidDynamic(PxTransform(pos.x, pos.y, pos.z));
    //PxShape* aConvexShape = PxRigidActorExt::createExclusiveShape(*body, PxConvexMeshGeometry(convexMesh), *material);
    PxShape* aConvexShape = pxScene.physics->createShape(PxConvexMeshGeometry(convexMesh), *material);
    body->setMass(10.f);
    body->attachShape(*aConvexShape);
    body->userData = rend;
    pinsBody.push_back(body);
    pxScene.scene->addActor(*body);
    aConvexShape->release();
}

void createDynamicSphere(PxRigidDynamic*& body, Renderable* rend, glm::vec3 const& pos, float radius)
{
    body = pxScene.physics->createRigidDynamic(PxTransform(pos.x, pos.y, pos.z));
    PxShape* sphereShape = pxScene.physics->createShape(PxSphereGeometry(radius), *ballMaterial);
    body->attachShape(*sphereShape);
    sphereShape->release();
    body->userData = rend;
    pxScene.scene->addActor(*body);
}

void initPhysicsScene()
{
    // material for ball and pins
    material = pxScene.physics->createMaterial(5.f, 5.f, 0.2f);
    ballMaterial = pxScene.physics->createMaterial(3.f, 10.f, 0.01f);
    // create ground
    bodyGround = pxScene.physics->createRigidStatic(PxTransformFromPlaneEquation(PxPlane(0, 1, 0, 0)));
    PxShape* planeShape = pxScene.physics->createShape(PxPlaneGeometry(), *material);
    bodyGround->attachShape(*planeShape);
    planeShape->release();
    bodyGround->userData = &rendGround;
    pxScene.scene->addActor(*bodyGround);

    // create ball
    createDynamicSphere(bodyHandle, &rendHandle, Objects::ball.pos, Objects::ball.size.x * 0.5f);
    bodyHandle->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
    PxRigidBodyExt::setMassAndUpdateInertia(*bodyHandle, 8.f);

    // create pins
    for (int i = 0; i < Objects::numPins; i++) {
        createDynamicPin(bodyPins[i], &rendPins[i], Objects::pins[i].pos, Objects::pins[i].size);
        PxRigidBodyExt::setMassAndUpdateInertia(*bodyPins[i], 0.2f);
    }


}

void updateTransforms()
{
    // Here we retrieve the current transforms of the objects from the physical simulation.
    auto actorFlags = PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC;
    PxU32 nbActors = pxScene.scene->getNbActors(actorFlags);
    if (nbActors)
    {
        vector<PxRigidActor*> actors(nbActors);
        pxScene.scene->getActors(actorFlags, (PxActor**)&actors[0], nbActors);
        for (auto actor : actors)
        {
            // We use the userData of the objects to set up the model matrices
            // of proper renderables.
            if (!actor->userData) continue;
            Renderable* renderable = (Renderable*)actor->userData;

            // get world matrix of the object (actor)
            PxMat44 transform = actor->getGlobalPose();
            auto& c0 = transform.column0;
            auto& c1 = transform.column1;
            auto& c2 = transform.column2;
            auto& c3 = transform.column3;

            // set up the model matrix used for the rendering
            renderable->physicsTransform = glm::mat4(
                c0.x, c0.y, c0.z, c0.w,
                c1.x, c1.y, c1.z, c1.w,
                c2.x, c2.y, c2.z, c2.w,
                c3.x, c3.y, c3.z, c3.w);
        }
    }
}

void moveHandle(float offset) {
    if (!bodyHandle) return;
    bodyHandle->setAngularVelocity(PxVec3(-camZ * 35.f, 0.f, offset));
}
bool blocked = false;
//maybe pass array of fallen pins and reset without them if provided.
void resetPinsAndBall(vector<int>downIndexes) {
    pinsBody.clear();
    startingPositions.clear();
    blocked = false;
    sort(downIndexes.begin(), downIndexes.end());
    renderables.clear();
    for (int i = 0; i < Objects::numPins; i++) {
        renderables.emplace_back(&rendGround);
        renderables.emplace_back(&rendHandle);
        if (!binary_search(downIndexes.begin(), downIndexes.end(), i)) {
            rendPins[i].model = &pinModel;
            rendPins[i].textureId = pinTexture;
            rendPins[i].localTransform = glm::scale(Objects::pins[i].size);
            renderables.emplace_back(&rendPins[i]);
            startingPositions.push_back(Objects::pins[i].pos.x);
        }
    }
    for (int i = 0; i < Objects::numPins; i++) {
        pxScene.scene->removeActor(*bodyPins[i]);
    }
    pxScene.scene->removeActor(*bodyHandle);

    // create ground
    bodyGround = pxScene.physics->createRigidStatic(PxTransformFromPlaneEquation(PxPlane(0, 1, 0, 0)));
    PxShape* planeShape = pxScene.physics->createShape(PxPlaneGeometry(), *material);
    bodyGround->attachShape(*planeShape);
    planeShape->release();
    bodyGround->userData = &rendGround;
    pxScene.scene->addActor(*bodyGround);

    // create ball
    createDynamicSphere(bodyHandle, &rendHandle, Objects::ball.pos, Objects::ball.size.x * 0.5f);
    bodyHandle->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
    PxRigidBodyExt::setMassAndUpdateInertia(*bodyHandle, 8.f);

    // create pins
    for (int i = 0; i < Objects::numPins; i++) {
        if (!binary_search(downIndexes.begin(), downIndexes.end(), i)) {
            createDynamicPin(bodyPins[i], &rendPins[i], Objects::pins[i].pos, Objects::pins[i].size);
            PxRigidBodyExt::setMassAndUpdateInertia(*bodyPins[i], 0.2f);
        }
    }
}
glm::mat4 createCameraMatrix()
{
    cameraDir = glm::normalize(glm::vec3(cosf(cameraAngle - glm::radians(90.0f)), 0, sinf(cameraAngle - glm::radians(90.0f))));
    glm::vec3 up = glm::vec3(0, 1, 0);
    cameraSide = glm::cross(cameraDir, up);
    return Core::createViewMatrix(cameraPos, cameraDir, up);
}
int vpress = 0;
int leftButtonState = 3;
void keyboard(unsigned char key, int x, int y)
{
    float angleSpeed = 0.1f;
    float moveSpeed = 2.f;
    switch (key)
    {
    case 'z': cameraAngle -= angleSpeed; break;
    case 'x': cameraAngle += angleSpeed; break;
    case 'w': cameraPos += cameraDir * moveSpeed; break;
    case 's': cameraPos -= cameraDir * moveSpeed; break;
    case 'd': cameraPos += cameraSide * moveSpeed; break;
    case 'a': cameraPos -= cameraSide * moveSpeed; break;
    case 'v':
        if (vpress == 0) {
            vpress = 1;
        }
        else {
            vpress = 0;
        }
        break;
    case 'r':
        resetPinsAndBall({});
        leftButtonState = 3;
        break;
    }
}
float differenceZ;
void mouse(int x, int y)
{
    ShowCursor(false);
    const float radius = 3.0f;
    const float sensitivity = 0.005f;
    differenceZ = lastPosition - x;
    camZ = (cos(x) / 100.f * radius - lastPosition + 300) * sensitivity;
    if (camZ > 4) {
        camZ = 4;
    }
    else if (camZ < -4) {
        camZ = -4;
    }
    view = glm::lookAt(glm::vec3(camX, cameraPos.y, camZ),
        Objects::ball.pos,
        glm::vec3(0.0f, 1.0f, 0.0f));
    lastPosition = x;
}
double startingTime;
double clickTime = 0.f;

double endTime;

void buttonClicks(int button, int state, int x, int y) {
    if (!blocked) {
        if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
            startingTime = glutGet(GLUT_ELAPSED_TIME);
            leftButtonState = GLUT_DOWN;
        }
        clickTime = glutGet(GLUT_ELAPSED_TIME) - startingTime;
        if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
            leftButtonState = GLUT_UP;
            endTime = fmod(clickTime, 600.f);
            blocked = true;
            moveHandle(-endTime);
            clickTime = 0;
        }
    }

}



void drawObjectColor(obj::Model* model, glm::mat4 modelMatrix, glm::vec3 color)
{
    GLuint program = programColor;

    glUseProgram(program);

    glUniform3f(glGetUniformLocation(program, "objectColor"), color.x, color.y, color.z);
    glUniform3f(glGetUniformLocation(program, "lightDir"), lightDir.x, lightDir.y, lightDir.z);

    glm::mat4 transformation = perspectiveMatrix * cameraMatrix * modelMatrix;
    glUniformMatrix4fv(glGetUniformLocation(program, "modelViewProjectionMatrix"), 1, GL_FALSE, (float*)&transformation);
    glUniformMatrix4fv(glGetUniformLocation(program, "modelMatrix"), 1, GL_FALSE, (float*)&modelMatrix);

    Core::DrawModel(model);

    glUseProgram(0);
}

void drawObjectTexture(obj::Model* model, glm::mat4 modelMatrix, GLuint textureId)
{
    GLuint program = programTexture;

    glUseProgram(program);

    glUniform3f(glGetUniformLocation(program, "lightDir"), lightDir.x, lightDir.y, lightDir.z);
    Core::SetActiveTexture(textureId, "textureSampler", program, 0);

    glm::mat4 transformation = perspectiveMatrix * cameraMatrix * modelMatrix;
    glUniformMatrix4fv(glGetUniformLocation(program, "modelViewProjectionMatrix"), 1, GL_FALSE, (float*)&transformation);
    glUniformMatrix4fv(glGetUniformLocation(program, "modelMatrix"), 1, GL_FALSE, (float*)&modelMatrix);

    Core::DrawModel(model);

    glUseProgram(0);
}
vector <bool> checked(10, false);
bool first = true;
double pinDownTimer = 0;
int throwNumber = 1;
int finalScore = 0;
void renderScene()
{
    double time = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    static double prevTime = time;
    double dtime = time - prevTime;
    prevTime = time;

    // Update physics
    if (dtime < 1.f) {
        physicsTimeToProcess += dtime;
        while (physicsTimeToProcess > 0) {
            // here we perform the physics simulation step
            pxScene.step(physicsStepTime);
            physicsTimeToProcess -= physicsStepTime;
        }
    }
    if (pinDownTimer != 0 && !first) {
        cout << time - pinDownTimer << '\n';
        if (time - pinDownTimer >= 2.f) {
            if (throwNumber % 2 == 0 || score == 10) {
                if (score == 10) {
                    throwNumber++;
                }
                resetPinsAndBall({});
                checked.clear();
                for (int i = 0; i < 10; i++) {
                    checked.push_back(false);
                }
                leftButtonState = 3;
                pinsDownIndexes.clear();
                finalScore += score;
                system("cls");
                cout << "Throw number: " << throwNumber << '\n';
                cout << "Round Score: " << score << '\n';
                cout << "Final Score: " << finalScore << '\n';
                score = 0;
            }
            else {
                resetPinsAndBall(pinsDownIndexes);
                leftButtonState = 3;
                system("cls");
                cout << "Throw number: " << throwNumber << '\n';
                cout << "First score: " << score << '\n';
            }
            pinDownTimer = 0;
            first = true;
            throwNumber++;
        }
    }
    // Update of camera and perspective matrices
    if (vpress == 0) {
        cameraMatrix = view;
        blocked = false;
    }
    else if (vpress == 1) {
        cameraMatrix = createCameraMatrix();
        blocked = true;
    }
    perspectiveMatrix = Core::createPerspectiveMatrix();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.1f, 0.3f, 1.0f);

    // update transforms from physics simulation
    updateTransforms();

    // render models
    for (Renderable* renderable : renderables) {
        drawObjectTexture(renderable->model, renderable->physicsTransform * renderable->localTransform, renderable->textureId);
    }


    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, 600, 600, 0.0, -1.0, 10.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_CULL_FACE);

    glClear(GL_DEPTH_BUFFER_BIT);



    glBegin(GL_QUADS);
    if (leftButtonState == GLUT_UP && blocked == false) {
        clickTime = endTime;
        blocked = true;
    }
    else if (leftButtonState == GLUT_DOWN) {
        clickTime = fmod(glutGet(GLUT_ELAPSED_TIME) - startingTime, 600.f);
    }
    else {
        clickTime = 0.0f;
    }
    if (clickTime < 300.f) {
        glColor3f(0.0f + (clickTime / 300.f), 1.0f, 0.0);
        glVertex2f(0.0, 0.0);
        glVertex2f(clickTime, 0.0);
        glVertex2f(clickTime, 2.f);
        glVertex2f(0.0, 2.f);
    }
    if (clickTime >= 300.f && clickTime < 560) {
        glColor3f(1.0f, 1.0f - (clickTime / 1500.f), 0.0);
        glVertex2f(0.0, 0.0);
        glVertex2f(clickTime, 0.0);
        glVertex2f(clickTime, 2.f);
        glVertex2f(0.0, 2.f);
    }
    if (clickTime >= 560) {
        glColor3f(1.0f, 0.0f, 0.0);
        glVertex2f(0.0, 0.0);
        glVertex2f(clickTime, 0.0);
        glVertex2f(clickTime, 2.f);
        glVertex2f(0.0, 2.f);
    }
    glEnd();
    for (int i = 0; i < pinsBody.size(); i++) {
        if (abs(startingPositions.at(i) - pinsBody.at(i)->getGlobalPose().p.x) >= 0.01 && !checked[i]) {
            pinsDownIndexes.push_back(i);
            score++;
            checked[i] = true;
            if (first) {
                pinDownTimer = time;
                first = false;
            }
        }
    }
    // Making sure we can render 3d again
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glutSwapBuffers();
}

void init()
{
    srand(time(0));
    glEnable(GL_DEPTH_TEST);
    programColor = shaderLoader.CreateProgram("shaders/shader_color.vert", "shaders/shader_color.frag");
    programTexture = shaderLoader.CreateProgram("shaders/shader_tex.vert", "shaders/shader_tex.frag");

    initRenderables();
    initPhysicsScene();


}

void shutdown()
{
    shaderLoader.DeleteProgram(programColor);
    shaderLoader.DeleteProgram(programTexture);
}

void idle()
{
    glutPostRedisplay();
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(200, 200);
    glutInitWindowSize(1000, 1000);
    glutCreateWindow("Bowling game for computer graphics");
    glewInit();

    init();
    glutKeyboardFunc(keyboard);
    glutPassiveMotionFunc(mouse);
    glutMouseFunc(buttonClicks);
    glutDisplayFunc(renderScene);
    glutIdleFunc(idle);

    glutMainLoop();

    shutdown();

    return 0;
}
