//
// Created by bruh on 17.11.15.
//

#include "ros/ros.h"

#include <QFile>
#include <QDebug>

#include "model/RoboyBehaviorXmlParser.h"

int main(int argc, char ** argv) {

    RoboyBehaviorMetadata metadata;
    metadata.m_ulBehaviorId = 1;
    metadata.m_sBehaviorName = "DefaultBehavior";

    RoboyBehaviorXmlParser parser;
    parser.readRoboyBehavior( metadata );

    return 0;
}
