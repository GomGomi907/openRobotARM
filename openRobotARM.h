#ifndef OPEN_ROBOT_ARM_H
#define OPEN_ROBOT_ARM_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef MAX_LINK
#define MAX_LINK 10
#endif

#define M_PI 3.14159265358979323846
#define M_RAD2DEG 0.01745329251994329576923690768489

typedef struct {
    double x;
    double y;
    double z;
} PointOrthogonal;

typedef struct {
    double r;
    double theta;
    double phi;
} PointSpherical;

typedef struct segmentLink {
    PointOrthogonal a;
    PointOrthogonal b;
    double len;
    double theta_offset, phi_offset;
    double theta_deg, phi_deg;
    struct segmentLink* prev;
    struct segmentLink* next;
} segmentLink;

segmentLink* createSegmentLink(double len, double theta_deg, double phi_deg, segmentLink* prev);
PointOrthogonal createPointOrthogonal(double x, double y, double z);
PointSpherical createPointSpherical(double r, double theta, double phi);
void calculateB(segmentLink* link);
void updateJointAngles(segmentLink* baseLink);
int calculateIK(segmentLink* baselink, PointOrthogonal* target, double epsilon, int maxIterations);
double distanceToEndpoint(segmentLink* link);
void adjustThetaJoint(segmentLink* link, double theta, double phi, double targetDistance);
void changeJoint(segmentLink* link, double theta, double phi_deg);

#endif
