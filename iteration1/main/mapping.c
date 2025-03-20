#include <stdio.h>
#include "mapping.h"

// Define the grid. This is our global occupancy grid.
int occupancy_grid[GRID_ROWS][GRID_COLS];

void init_occupancy_grid(void) {
    for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {
            occupancy_grid[i][j] = 0;
        }
    }
}

void update_occupancy_grid(float distance, int row) {
    // With a resolution of 0.1m, convert the distance (in meters) to a column index.
    int cell_index = (int)(distance / 0.1);
    // Check that the calculated index and row are within the grid bounds.
    if (cell_index >= 0 && cell_index < GRID_COLS && row >= 0 && row < GRID_ROWS) {
        occupancy_grid[row][cell_index] = 1;
    }
}

void print_occupancy_grid(void) {
    printf("Occupancy Grid:\n");
    for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {
            printf("%d ", occupancy_grid[i][j]);
        }
        printf("\n");
    }
}

int (*getGrid(void))[GRID_COLS] {
    return occupancy_grid;
}
