#include <stdio.h>
#include "openRobotARM.h"

#define MAX_LINK 4

int main() {
    // Create the base link
    segmentLink* Link[MAX_LINK];
    Link[0] = createSegmentLink(10.0, 50.0, 10.0, NULL);
    Link[1] = createSegmentLink(10.0, 40.0, 20.0, Link[0]);
    Link[2] = createSegmentLink(10.0, 30.0, 30.0, Link[1]);
    Link[3] = createSegmentLink(10.0, 20.0, 40.0, Link[2]);

    // Create the endpoint
    PointOrthogonal endpoint = {32.251050, 5.686730, -6.232568};

    // Calculate the coordinates of b for each link
    calculateB(Link[0]);

    // Print out the coordinates of a and b for each link
    printf("calculate endposition:\n");
    for (int i = 0; i < MAX_LINK; i++) {
        printf("Link[%d] a: (%f, %f, %f)\n", i, Link[i]->a.x, Link[i]->a.y, Link[i]->a.z);
        printf("Link[%d] b: (%f, %f, %f)\n", i, Link[i]->b.x, Link[i]->b.y, Link[i]->b.z);
    }
    printf("\n");

    calculateB(Link[0]); // Update link coordinates
    for (int i = 0; i < MAX_LINK; i++) {
        printf("Link[%d] theta: %f\n", i, Link[i]->theta_deg);
        printf("Link[%d] phi: %f\n", i, Link[i]->phi_deg);
    }
    printf("\n");
    for (int i = 0; i < MAX_LINK; i++) {
        changeJoint(Link[i], 0.0, 0.0);
        calculateB(Link[0]);
    }
    printf("zero-init\n");
    for (int i = 0; i < MAX_LINK; i++) {
        printf("Link[%d] a: (%f, %f, %f)\n", i, Link[i]->a.x, Link[i]->a.y, Link[i]->a.z);
        printf("Link[%d] b: (%f, %f, %f)\n", i, Link[i]->b.x, Link[i]->b.y, Link[i]->b.z);
    }
    printf("\n");
    /*
    // Apply inverse kinematics
    calculateIK(Link[0], &endpoint, 0.0001, 10);

    // Print out the coordinates of a and b for each link
    printf("After IK:\n");
    for (int i = 0; i < MAX_LINK; i++) {
        printf("Link[%d] a: (%f, %f, %f)\n", i, Link[i]->a.x, Link[i]->a.y, Link[i]->a.z);
        printf("Link[%d] b: (%f, %f, %f)\n", i, Link[i]->b.x, Link[i]->b.y, Link[i]->b.z);
    }
    printf("\n");

    // Print out the angles for each link
    for (int i = 0; i < MAX_LINK; i++) {
        printf("Link[%d] theta: %f\n", i, Link[i]->theta_deg);
        printf("Link[%d] phi: %f\n", i, Link[i]->phi_deg);
    }
    printf("\n");

    calculateB(Link[0]); // Update link coordinates
    printf("check :\n");
    for (int i = 0; i < MAX_LINK; i++) {
        printf("Link[%d] a: (%f, %f, %f)\n", i, Link[i]->a.x, Link[i]->a.y, Link[i]->a.z);
        printf("Link[%d] b: (%f, %f, %f)\n", i, Link[i]->b.x, Link[i]->b.y, Link[i]->b.z);
    }
    printf("\n");
    */
    // Free the memory allocated for the links
    for (int i = 0; i < MAX_LINK; i++) {
        free(Link[i]);
    }
    return 0;
}
