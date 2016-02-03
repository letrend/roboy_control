//
// Created by bruh on 12/14/15.
//

#include <QDebug>
#include <QProcess>
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

#include "DataTypes.h"

QList<ROSController> m_listControllers;
QMap<qint32, ros::NodeHandle *> m_mapNodeHandles;

QString fileName = "ControllerStub.launch";

QObject * parent;

bool callbackInitialize(common_utilities::Initialize::Request & req, common_utilities::Initialize::Response & res);
bool startNode(qint32 id, QString name);

int main(int argc, char ** argv) {
    qDebug() << "Test node started ...";

    ros::init(argc, argv, "roboy_control_test_node");
    ros::NodeHandle m_nodeHandle;

    ros::ServiceServer initializeServer = m_nodeHandle.advertiseService("roboy/initialize", callbackInitialize);

    ros::spin();
}

bool callbackInitialize(common_utilities::Initialize::Request & req, common_utilities::Initialize::Response & res){
    qDebug() << "Process 'initialize' request";
    qDebug() << "Build 'initialize' response";

    ROSController controller;
    common_utilities::ControllerState responseMessage;
    for(qint8 id : req.idList) {
        QString serviceName;
        serviceName.sprintf("roboy/trajectory_motor%i", id);

        controller.id = id;

        if(startNode(id, serviceName)){
            controller.state = STATUS::INITIALIZED;
        } else {
            controller.state = STATUS::INITIALIZE_ERROR;
        }

        m_listControllers.append(controller);

        responseMessage.id = id;
        responseMessage.state = controller.state;
        res.states.push_back(responseMessage);

        qDebug() << "\t- Update Controller: " << controller.toString();
    }

    return true;
}

void callbackSteering(const common_utilities::Steer & msg){
    qDebug() << "Heard steering message: " << msg.steeringCommand;
}

bool startNode(qint32 id, QString name) {
    QString roboyControlHome = QProcessEnvironment::systemEnvironment().value("ROBOY_TEST_HOME");
    if(roboyControlHome.isEmpty()) {
        qDebug() << "Set environment variable 'ROBOY_TEST_HOME' to etc folder.";
        return false;
    }

    QXmlStreamWriter writer;
    QString launchFile = roboyControlHome + "/etc/" + fileName;
    QFile file(launchFile);

    if (file.exists())
        file.remove();

    file.open(QFile::ReadWrite | QFile::Text);
    writer.setDevice(&file);
    writer.setAutoFormatting(true);

    QString nodeName;
    nodeName.sprintf("controller_stub_%i", id);

    writer.writeStartElement("launch");
    writer.writeStartElement("node");
    writer.writeAttribute("name", nodeName);
    writer.writeAttribute("pkg", "roboy_control_test");
    writer.writeAttribute("type", "controller_stub");
    writer.writeAttribute("args", name);
    writer.writeEndElement();
    writer.writeEndElement();

    file.close();

    QString startCommand = "sh " + roboyControlHome + "/etc/StartNode.sh";
    qDebug() << "Run Node start script: " << startCommand;

    if(QProcess::startDetached(startCommand)){
        return true;
    }
    return false;
}