#!/usr/bin/env python3

"""Shared final-demo map and route constants for path planning."""

WIDTH_M = 15.0
HEIGHT_M = 10.8

PLANTS = [
    ("P1", 1.00, 10.00),
    ("P2", 5.00, 4.00),
    ("P3", 8.40, 8.00),
    ("P4", 10.00, 5.00),
    ("P5", 12.40, 1.00),
    ("P6", 14.60, 8.00),
]

MARKERS = [
    (1, 0.00, 1.00, 0.30, 0.0),
    (2, 0.00, 3.60, 0.30, 0.0),
    (3, 0.00, 6.60, 0.30, 0.0),
    (4, 0.00, 9.60, 0.30, 0.0),
    (5, 15.00, 1.00, 0.30, 180.0),
    (6, 15.00, 3.60, 0.30, 180.0),
    (7, 15.00, 6.60, 0.30, 180.0),
    (8, 15.00, 9.60, 0.30, 180.0),
    (9, 5.00, 0.00, 0.30, 90.0),
    (10, 8.00, 0.00, 0.30, 90.0),
    (11, 11.00, 0.00, 0.30, 90.0),
    (12, 14.00, 0.00, 0.30, 90.0),
    (13, 5.00, 10.80, 0.30, -90.0),
    (14, 8.00, 10.80, 0.30, -90.0),
    (15, 11.00, 10.80, 0.30, -90.0),
    (16, 14.00, 10.80, 0.30, -90.0),
    (18, 8.20, 5.20, 0.30, 0.0),
    (19, 7.80, 5.20, 0.30, 180.0),
    (20, 8.00, 5.40, 0.30, 90.0),
    (22, 8.00, 5.00, 0.30, -90.0),
    (27, 1.50, 3.60, 0.30, 90.0),
    (29, 8.00, 1.30, 0.30, 180.0),
]

STOP_POINTS = [
    ("P1", 1.00, 9.30),
    ("P2", 5.50, 5.30),
    ("P3", 8.40, 6.75),
    ("P4", 10.50, 6.20),
    ("P6", 14.50, 7.00),
    ("P5", 10.90, 1.00),
]

COVERAGE_STOPS = []

ROUTE_WAYPOINTS = [
    ("START",               0.50,  0.50),
    ("LEFT_CORRIDOR_ENTRY", 1.00,  0.50),
    ("P1_STOP",             1.00,  9.30),
    ("LEFT_RETURN_TOP",     0.70,  9.30),
    ("LEFT_RETURN_GAP",     0.70,  6.75),
    ("LEFT_CORRIDOR_GAP",   1.00,  6.75),
    ("P2_BRANCH_ENTRY",     5.00,  6.75),
    ("P2_STOP",             5.50,  5.30),
    ("P2_M19_SWEEP",        5.30,  4.80),
    ("P2_RETURN_SIDE",      5.30,  5.85),
    ("P3_LANE_ENTRY",       8.40,  5.85),
    ("P3_STOP",             8.40,  6.75),
    ("P4_LANE_ENTRY",       10.00, 6.75),
    ("P4_STOP",             10.50, 6.20),
    ("P4_EXIT_EAST",        10.80, 6.20),
    ("P4_RIGHT_LANE_UP",    10.80, 6.55),
    ("P6_SAFE_LANE",        14.20, 6.55),
    ("P6_CAMERA_ALIGN",     14.50, 6.50),
    ("P6_STOP",             14.50, 7.00),
    ("RIGHT_SIDE_P5_LANE",  14.20, 1.00),
    ("P5_STOP",             10.90, 1.00),
    ("P5_EXIT_BOTTOM",      10.90, 0.50),
    ("BOTTOM_HOME",          0.50, 0.50),
    ("FINISH",               0.50, 0.50),
]

TABLE_SIZE_X = 1.45
TABLE_SIZE_Y = 1.45

TABLES = [
    ("T1", 3.45, 8.25, TABLE_SIZE_X, TABLE_SIZE_Y),
    ("T2", 3.45, 5.25, TABLE_SIZE_X, TABLE_SIZE_Y),
    ("T3", 3.45, 2.25, TABLE_SIZE_X, TABLE_SIZE_Y),
    ("T4", 6.45, 8.25, TABLE_SIZE_X, TABLE_SIZE_Y),
    ("T5", 6.45, 2.25, TABLE_SIZE_X, TABLE_SIZE_Y),
    ("T6", 9.85, 8.25, TABLE_SIZE_X, TABLE_SIZE_Y),
    ("T7", 9.85, 2.25, TABLE_SIZE_X, TABLE_SIZE_Y),
    ("T8", 13.15, 8.25, TABLE_SIZE_X, TABLE_SIZE_Y),
    ("T9", 13.15, 5.25, TABLE_SIZE_X, TABLE_SIZE_Y),
    ("T10", 13.15, 2.25, TABLE_SIZE_X, TABLE_SIZE_Y),
]

LECTERNS = [
    # The lectern backs onto the positive-Y wall; keep the front edge at
    # y=9.675 m and make the back side solid to the room boundary.
    ("Lectern", 2.60, 10.2375, 1.45, 1.125),
]
