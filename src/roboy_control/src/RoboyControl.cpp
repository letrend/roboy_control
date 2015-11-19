//
// Created by bruh on 17.11.15.
//

#include "ros/ros.h"

#include <QFile>
#include <QDebug>

#include "model/RoboyBehaviorXmlParser.h"
#include "model/XmlModelService.h"

int main(int argc, char ** argv) {

    RoboyBehavior behavior;
    RoboyBehaviorMetadata metadata;
    metadata.m_ulBehaviorId = 1;
    metadata.m_sBehaviorName = "DefaultBehavior";

    behavior.m_metadata = metadata;

    RoboyBehaviorXmlParser parser;
    parser.readRoboyBehavior( &behavior );

    LOG << behavior.toString();

    XmlModelService xmlModelService;
    IModelService & modelService = xmlModelService;

    modelService.getBehavior(metadata);

    return 0;
}
