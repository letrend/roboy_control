//
// Created by bruh on 12/14/15.
//

#include <QDebug>
#include <DataTypes.h>
#include <QtCore/qprocess.h>
#include <QtCore/qxmlstream.h>
#include <QtCore/qfile.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "common_utilities/Initialize.h"
#include "common_utilities/ControllerState.h"
//#include "common_utilities/InitializeRequest.h"
//#include "common_utilities/InitializeResponse.h"
#include "common_utilities/Trajectory.h"
#include "common_utilities/Steer.h"
#include "common_utilities/Status.h"

#include "CommonDefinitions.h"

ros::Publisher publisher;
ros::Publisher publisherStatus;
ros::Subscriber subscriber;
ros::Subscriber subscriberMotor1;
ros::Subscriber subscriberSteer;

bool callbackMotor(common_utilities::Trajectory::Request & req, common_utilities::Trajectory::Response & res);
void startNode(QString name);

QString fileName = "ControllerStub.launch";

QList<ROSController> m_listControllers;
QMap<qint32, ros::NodeHandle *> m_mapNodeHandles;

bool callbackInitialize(common_utilities::Initialize::Request & req, common_utilities::Initialize::Response & res){
    qDebug() << "Process 'initialize' request";
    qDebug() << "Build 'initialize' response";

    ROSController controller;
    common_utilities::ControllerState responseMessage;
    for(qint8 id : req.idList) {
        controller.id = id;
        controller.state = STATUS::INITIALIZED;
        m_listControllers.append(controller);

        QString serviceName;
        serviceName.sprintf("motor%i", id);

//        startNode(serviceName);

        responseMessage.id = id;
        responseMessage.state = 1;
        res.states.push_back(responseMessage);

        qDebug() << "\t- Update Controller: " << controller.toString();
    }

    return true;
}


void callbackSteering(const common_utilities::Steer & msg){
    qDebug() << "Heard steering message: " << msg.steeringCommand;
}


int main(int argc, char ** argv) {
    qDebug() << "Test node started ...";

    ros::init(argc, argv, "roboy_control_test_node");
    ros::NodeHandle m_nodeHandle;
    ros::NodeHandle m_nodeHandle2;

    ros::ServiceServer initializeServer = m_nodeHandle.advertiseService("roboy/initialize", callbackInitialize);

    //ros::Subscriber steeringSubscriber = m_nodeHandle.subscribe("steering", 1000, callbackSteering);

    ros::spin();
}

void startNode(QString name) {
    QString roboyControlHome = QProcessEnvironment::systemEnvironment().value("ROBOY_CONTROL_HOME");

    QXmlStreamWriter writer;
    QString launchFile = roboyControlHome + "/etc/" + fileName;
    QFile file(launchFile);

    if (file.exists())
        file.remove();

    file.open(QFile::ReadWrite | QFile::Text);
    writer.setDevice(&file);
    writer.setAutoFormatting(true);

    writer.writeStartElement("launch");
    writer.writeStartElement("node");
    writer.writeAttribute("name", "controller_stub");
    writer.writeAttribute("pkg", "roboy_control");
    writer.writeAttribute("type", "controller_stub");
    writer.writeAttribute("args", name);
    writer.writeEndElement();
    writer.writeEndElement();

    file.close();

    QString startScript = roboyControlHome + "/etc/StartNode.sh";
    qDebug() << "Run Node start script: " << startScript;
    pid_t pid = fork();
    if (pid == 0) {
        // CHILD
        system(startScript.toStdString().data());

    } else if (pid < 0) {
        qDebug() << "Fork failed";
    } else {
        // RARENT
    }
}