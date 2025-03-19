#ifndef MAPPING_H
#define MAPPING_H

#define GRID_ROWS 5
#define GRID_COLS 5

// The occupancy grid is a 5x5 grid representing a 0.5m x 0.5m area with 0.1m resolution.
// 0 indicates a free cell and 1 indicates an occupied cell.
extern int occupancy_grid[GRID_ROWS][GRID_COLS];

// Initializes the occupancy grid (sets all cells to 0).
void init_occupancy_grid(void);

// Updates the occupancy grid for a given row based on a sensor reading converted to distance (in meters).
// The function calculates which column to update based on the distance.
void update_occupancy_grid(float distance, int row);

// Prints the occupancy grid to the serial console for debugging.
void print_occupancy_grid(void);


int (*getGrid(void))[GRID_COLS];

#endif // MAPPING_H
