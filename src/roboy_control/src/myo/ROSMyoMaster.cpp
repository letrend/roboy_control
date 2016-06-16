#include "ROSMyoMaster.h"

//#include "controller_manager_msgs/SwitchController.h"
//#include "controller_manager_msgs/UnloadController.h"

ROSMyoMaster::ROSMyoMaster() {

    m_initializePublisher = m_nodeHandle.advertise<common_utilities::Initialize>("/roboy/initialize", 1000);
    m_steerPublisher = m_nodeHandle.advertise<common_utilities::Steer>("/roboy/steer", 1000);

//    m_recordClient = m_nodeHandle.serviceClient<common_utilities::Record>("/roboy/record");
//    m_recordSteerPublisher = m_nodeHandle.advertise<common_utilities::Steer>("/roboy/steer_record", 1000);

//    m_switchController = m_nodeHandle.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
//    m_unloadController = m_nodeHandle.serviceClient<controller_manager_msgs::UnloadController>("/controller_manager/unload_controller");
}

void ROSMyoMaster::sendInitializeRequest(const QList<IMotorController *> initializationList) const {
    TRANSCEIVER_LOG << "Trigger 'Send Initialize'";

    // TODO: make this a counter for whoever is responsible in your crazy architecture (and by crazy I mean suuuuper nice of course)
    static int idCounter = 1;

    common_utilities::Initialize initialize;
    for(IMotorController * motor : initializationList) {
        common_utilities::ControllerRequest request;
        //request.id = idCounter;
        request.id = motor->getId();
        request.controlmode = motor->getControlMode();
        char resource[20];
        sprintf(resource, "motor%d", request.id);
        request.resource = resource;
        request.ganglion = 0;
        request.motor = 0;
        initialize.controllers.push_back(request);

        idCounter++;
    }

    m_initializePublisher.publish(initialize);
}

void ROSMyoMaster::sendSteeringMessage(const SteeringCommand command) const {
    TRANSCEIVER_LOG << "Sending steering message: " << command << " to " << m_steerPublisher.getNumSubscribers() << " subscribers.";

    common_utilities::Steer steer;
    steer.steeringCommand = command;

    m_steerPublisher.publish(steer);
}

void ROSMyoMaster::startRecording(const QMap<qint32, IMotorController *> controllers, qint32 sampleRate) const {
/*    TRANSCEIVER_LOG << "Start Recording: SampleRate: " << m_sampleRate;
    bool res = false;

    common_utilities::Record message;
    message.request.sampleRate = m_sampleRate;

    common_utilities::ControllerRequest controllerRequest;
    for(auto controller : m_recordRequest.values()) {
        TRANSCEIVER_LOG << "Request Controller: " << controller->m_id;
        controllerRequest.id = controller->m_id;
        controllerRequest.controlmode = controller->m_controlMode;
        message.request.controllers.push_back(controllerRequest);
    }

    res = m_recordClient.call(message);
    if(res) {
        TRANSCEIVER_LOG << "Record successful";
        RoboyBehavior * behavior = new RoboyBehavior();
        Trajectory trajectory;
        RoboyWaypoint waypoint;

        qint32 i = 0;
        for(auto t : message.response.trajectories) {
            trajectory.m_listWaypoints.clear();
            trajectory.m_sampleRate = (qint32) t.samplerate;
            trajectory.m_controlMode = m_recordRequest[t.id]->m_controlMode;
            for(auto wp : t.waypoints) {
                waypoint.m_ulValue = wp;
                trajectory.m_listWaypoints.append(waypoint);
            }
            behavior->m_mapMotorTrajectory.insert(t.id, trajectory);
            TRANSCEIVER_LOG << "Trajectory : " << i++ << " motor-id: " << t.id << " waypoints: " << t.waypoints.size() << " sampleRate: " << trajectory.m_sampleRate << " ControlMode: " << trajectory.m_controlMode;
        }

        m_mutexData.lock();
        m_pRecordedBehavior = behavior;
        m_mutexData.unlock();

        res = true;
    } else {
        TRANSCEIVER_WAR << "Record failed";
        m_mutexData.lock();
        m_pRecordedBehavior = nullptr;
        m_mutexData.unlock();
        res = false;
    }
    emit signalRecordFinished(res);*/

}

void ROSMyoMaster::sendRecordSteeringMessage(SteeringCommand command) const {
/*    TRANSCEIVER_LOG << "Send Steering Command " << m_recordSteeringCommand;

    common_utilities::Steer message;
    message.steeringCommand = m_recordSteeringCommand;
    m_recordSteerPublisher.publish(message);
*/
}