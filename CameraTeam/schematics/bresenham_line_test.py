import matplotlib.pyplot as plt
def bresenham_line_on_bitmap(x1, y1, x2, y2, bitmap, fill_value=1):
    """
    Uses Bresenham's line algorithm to fill boxes on a bitmap along a line 
    from (x1, y1) to (x2, y2).
    
    Args:
        x1, y1: Starting coordinates of the line.
        x2, y2: Ending coordinates of the line.
        bitmap: 2D array (list of lists) representing the bitmap.
        fill_value: Value to fill the bitmap cells (default is 1).

    Returns:
        Modified bitmap with the line filled.
    """
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        # Mark the cell on the bitmap
        bitmap[y1][x1] = fill_value
        if x1 == x2 and y1 == y2:  # End condition
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy

    return bitmap


# Example Usage
if __name__ == "__main__":
    # Create a blank bitmap (e.g., 10x10 grid initialized with 0s)
    width, height = 15, 10
    bitmap = [[0 for _ in range(width)] for _ in range(height)]

    # Define start and end points of the line
    x_start, y_start = 2, 3
    x_end, y_end = 12, 7

    # Fill the line on the bitmap
    bresenham_line_on_bitmap(x_start, y_start, x_end, y_end, bitmap)

    # Print the bitmap
    for row in bitmap:
        print(" ".join(str(cell) for cell in row))

    # Optional: Visualize the bitmap using matplotlib


plt.imshow(bitmap, cmap="Greys", origin="lower")
plt.title("Bresenham Line on Bitmap")
plt.show()


