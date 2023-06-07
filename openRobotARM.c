#include <math.h>
#include <stdlib.h>
#include <stddef.h>
#include "openRobotARM.h"

void calculateB(segmentLink *link) {
    if (link == NULL) {
        return;
    }
    if (link->prev != NULL)
    {
        link->a.x = link->prev->b.x;
        link->a.y = link->prev->b.y;
        link->a.z = link->prev->b.z;
        link->theta_offset = link->prev->theta_offset + link->prev->theta_deg;
        link->phi_offset = link->prev->phi_offset + link->prev->phi_deg;
    }
    link->b.x = link->a.x + link->len * sin(M_RAD2DEG * (link->theta_offset + link->theta_deg)) * cos(M_RAD2DEG * (link->phi_offset + link->phi_deg));
    link->b.y = link->a.y + link->len * sin(M_RAD2DEG * (link->theta_offset + link->theta_deg)) * sin(M_RAD2DEG * (link->phi_offset + link->phi_deg));
    link->b.z = link->a.z + link->len * cos(M_RAD2DEG * (link->theta_offset + link->theta_deg));
    calculateB(link->next);
    return;
}

double distanceToEndpoint(segmentLink* link) {
    if (link->next == NULL) {
        // 맨 끝 링크인 경우
        double dx = link->b.x - link->a.x;
        double dy = link->b.y - link->a.y;
        double dz = link->b.z - link->a.z;
        return sqrt(dx * dx + dy * dy + dz * dz);
    } else {
        // 다음 링크까지의 거리와 현재 링크의 길이를 합산하여 반환
        double dx = link->next->a.x - link->a.x;
        double dy = link->next->a.y - link->a.y;
        double dz = link->next->a.z - link->a.z;
        double distToNext = sqrt(dx * dx + dy * dy + dz * dz);
        return distToNext + distanceToEndpoint(link->next);
    }
}

double calculateTheta(segmentLink* link) {
    return 180-asin(link->b.z/link->len);
}

double calculatePhi(segmentLink* link) {
    return 180-asin(link->b.y/link->len);
}

void backwardIteration(segmentLink* link, PointOrthogonal* target, double totalLength) {
    segmentLink* currLink = link;
    double scaleFactor = totalLength / distanceToEndpoint(link);
    while (currLink->next != NULL) {
        currLink = currLink->next;
    }
    
    while (currLink != NULL) {
        currLink->b.x = target->x;
        currLink->b.y = target->y;
        currLink->b.z = target->z;

        scaleFactor = distanceToEndpoint(currLink);
        printf("%f\n", scaleFactor);

        // 현재 링크의 a 좌표를 계산

        currLink->a.x = currLink->b.x - currLink->len;
        currLink->a.y = currLink->b.y - currLink->len;
        currLink->a.z = currLink->b.z - currLink->len;

        printf("a\n");
        printf("Link[] a: (%f, %f, %f)\n", currLink->a.x, currLink->a.y, currLink->a.z);
        printf("Link[] b: (%f, %f, %f)\n", currLink->b.x, currLink->b.y, currLink->b.z);
        printf("\n");

        target->x = currLink->a.x;
        target->y = currLink->a.y;
        target->z = currLink->a.z;

        printf("backward\n");
        printf("Link[] a: (%f, %f, %f)\n", currLink->a.x, currLink->a.y, currLink->a.z);
        printf("Link[] b: (%f, %f, %f)\n", currLink->b.x, currLink->b.y, currLink->b.z);
        printf("\n");

        currLink = currLink->prev;
    }
}


void forwardIteration(segmentLink* link, PointOrthogonal* target, double totalLength) {
    segmentLink* currLink = link;
    double scaleFactor = totalLength / distanceToEndpoint(link);

    while (currLink != NULL) {
        currLink->b.x = currLink->b.x + scaleFactor * (target->x - currLink->b.x);
        currLink->b.y = currLink->b.y + scaleFactor * (target->y - currLink->b.y);
        currLink->b.z = currLink->b.z + scaleFactor * (target->z - currLink->b.z);
        currLink->next->a.x = currLink->b.x;
        currLink->next->a.y = currLink->b.y;
        currLink->next->a.z = currLink->b.z;
        currLink = currLink->next;
    }
}

int calculateIK(segmentLink* baselink, PointOrthogonal* target, double epsilon, int maxIterations) {
    double totalLength = distanceToEndpoint(baselink);
    for (int i = 0; i < maxIterations; i++) {
        //forwardIteration(baselink, target, totalLength);
        backwardIteration(baselink, target, totalLength);
        while(baselink != NULL) {
            printf("Link[%d] a: (%f, %f, %f)\n", i, baselink->a.x, baselink->a.y, baselink->a.z);
            printf("Link[%d] b: (%f, %f, %f)\n", i, baselink->b.x, baselink->b.y, baselink->b.z);
            baselink = baselink->next;
        }
        printf("\n");

        double error = distanceToEndpoint(baselink);
        if (error < epsilon) {
            return 0;  // 수렴 성공
        }
    }

    return -1;  // 수렴 실패
}
segmentLink *createSegmentLink(double len, double theta_deg, double phi_deg, segmentLink *prev) {
    segmentLink *newLink = (segmentLink*)malloc(sizeof(segmentLink));
    if (newLink == NULL) {
        printf("Memory allocation failed\n");
        return NULL;
    }
    
    newLink->len = len;
    newLink->theta_deg = theta_deg;
    newLink->phi_deg = phi_deg;
    newLink->prev = prev;
    newLink->next = NULL;
    
    if (prev != NULL) {
        prev->next = newLink;
        newLink->theta_offset = prev->theta_offset + prev->theta_deg;
        newLink->phi_offset = prev->phi_offset + prev->phi_deg;
    }
    else {
        newLink->a.x = 0.0;
        newLink->a.y = 0.0;
        newLink->a.z = 0.0;
        newLink->theta_offset = 0.0;
        newLink->phi_offset = 0.0;
    }
    calculateB(newLink);
    return newLink;
}

PointOrthogonal createPointOrthogonal(double x, double y, double z) {
    PointOrthogonal newPoint;
    newPoint.x = x;
    newPoint.y = y;
    newPoint.z = z;
    return newPoint;
}

void changeJoint(segmentLink *link, double theta, double phi_deg) {
    if (link == NULL) {
        printf("Link is null\n");
        return;
    }
    // Update the angles of the link

    link->theta_deg = theta;
    link->phi_deg = phi_deg;

    // Update the coordinates of b
    calculateB(link);
    if (link->next != NULL)
        changeJoint(link->next, link->next->theta_deg, link->next->phi_deg);
    return;
}