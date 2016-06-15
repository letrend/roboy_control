#ifndef DATATYPES_H
#define DATATYPES_H

#include "CommonDefinitions.h"

#include <QDebug>
#include <QString>
#include <QMap>
#include <QObject>
#include <QtQml>

class IModelService;

class PlayerStateClass : public QObject
{
    Q_OBJECT

    public:
        enum PlayerState {  
            PLAYER_NOT_READY = 0,
            PLAYER_READY,
            // Preprocessing: Success, Failures
            PLAYER_PREPROCESSING,
            PLAYER_PREPROCESS_FAILED_EMPTY,
            PLAYER_PREPROCESS_FAILED_LOAD_BEHAVIOR,
            PLAYER_PREPROCESS_FAILED_MODE_CONFLICT,
            PLAYER_PREPROCESS_FAILED_CONTROLLER_STATE_CONFLICT,
            PLAYER_PREPROCESS_FAILED_SAMPLERATE_CONFLICT,
            PLAYER_PREPROCESS_FAILED_OVERLAPPING,
            PLAYER_PREPROCESS_FAILED_COMMUNICATION_TIMEOUT,
            PLAYER_PREPROCESS_SUCCEEDED,
            //
            PLAYER_TRAJECTORY_READY,
            PLAYER_PLAYING,
            PLAYER_PAUSED 
        };

        Q_ENUMS(PlayerState)

        static void registerPlayerStates() {
            qmlRegisterType<PlayerStateClass>("PlayerState", 1, 0, "PlayerState");
        }
};

typedef PlayerStateClass::PlayerState PlayerState;

class RecorderStateClass : public QObject
{
    Q_OBJECT

    public:
        enum RecorderState {
            RECORDER_NOT_READY = 0,
            RECORDER_READY,
            RECORDER_RECORDING,
            RECORDER_FINISHED_RECORDING
        };

        Q_ENUMS(RecorderState)

        static void registerRecorderStates() {
            qmlRegisterType<RecorderStateClass>("RecorderState", 1, 0, "RecorderState");
        }
};

typedef RecorderStateClass::RecorderState RecorderState;

struct RoboyBehaviorMetadata {
    quint64   m_ulBehaviorId;
    QString   m_sBehaviorName;
};

struct RoboyWaypoint {
    float   m_ulValue;
};

struct Trajectory {
    ControlMode           m_controlMode;
    qint32                m_sampleRate;
    QList<RoboyWaypoint>  m_listWaypoints;
    qint32 getDuration() {
        return m_listWaypoints.count() * m_sampleRate;
    }

    QString toString() const {
        QString string;
        string.sprintf("TRAJECTORY [controlMode:%i][sampleRate:%i][wp-count:%i]",
                       (int) m_controlMode,
                       (int) m_sampleRate,
                       m_listWaypoints.size());
        return string;
    }
};

struct RoboyBehavior {
    RoboyBehaviorMetadata       m_metadata;
    QMap<qint32, Trajectory>    m_mapMotorTrajectory;

    quint64 getDuration() const {
        int maxDuration = 0;
        int currentDuration = 0;
        for (Trajectory trajectory : m_mapMotorTrajectory) {
            currentDuration = trajectory.getDuration();
            if(currentDuration > maxDuration)
                maxDuration = currentDuration;
        }
        return maxDuration;
    }

    QString toString() {
        QString string;
        string.sprintf("RoboyBehavior={Id=%lu,Name=%s,Motor-Count=%i}",
                       (unsigned long) m_metadata.m_ulBehaviorId,
                       m_metadata.m_sBehaviorName.toLatin1().data(),
                       m_mapMotorTrajectory.count());

        return string;
    }

};

struct RoboyBehaviorMetaExecution {
    qint64                 lId;
    qint64                 lTimestamp;
    RoboyBehaviorMetadata   behaviorMetadata;
};

struct RoboyBehaviorExecution{
    qint64                  lId;
    qint64                  lTimestamp;
    RoboyBehavior           behavior;

    qint64 getEndTimestamp() {
        return lTimestamp + behavior.getDuration();
    }
};

struct RoboyBehaviorMetaplan {
    QList<RoboyBehaviorMetaExecution>   listExecutions;
};

struct RoboyBehaviorPlan {

private:
    QList<RoboyBehaviorExecution>       m_listExecutions;
    QMap<qint32, Trajectory>            m_mapMotorTrajectories;

    bool    m_bLoadedCompletely = false;

    qint64  m_startTimestamp = -1;
    qint64  m_endTimestamp = -1;

    qint32      m_sampleRate;


public:
    RoboyBehaviorPlan(IModelService * modelService, const RoboyBehaviorMetaplan & metaPlan);

    bool doFlattening();
    bool isEmpty() const;
    bool isLoadedCompletely() const;

    qint64 getStartTimestamp() const;
    qint64 getEndTimestamp() const;
    qint64 getDuration() const;
    const QList<RoboyBehaviorExecution> & getExecutionsList() const;
    QMap<qint32, Trajectory> getTrajectories() const;

private:
    void setStartTimestamp();
    void setEndTimestamp();

    bool insertExecution(RoboyBehaviorExecution & execution);

    void printMap() const;
};

#endif // DATATYPES_H
