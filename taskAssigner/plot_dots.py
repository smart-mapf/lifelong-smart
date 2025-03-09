import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont
from sklearn.cluster import KMeans

def get_character_pixels(character, grid_size=50, num_dots=200):
    """Generate exactly 20 evenly spaced pixel positions to form a given character."""
    img_size = 100  # Larger canvas for better resolution
    font_size = 120  # Adjust font size to fit in the image
    
    # Create a blank image with white background
    img = Image.new("L", (img_size, img_size), color=255)
    draw = ImageDraw.Draw(img)

    # Load a font (default PIL font if no TTF available)
    try:
        font = ImageFont.truetype("arial.ttf", font_size)
    except IOError:
        font = ImageFont.load_default(font_size)

    # Get character bounding box and center it
    text_size = draw.textbbox((0, 0), character, font=font)
    text_w = text_size[2] - text_size[0]
    text_h = text_size[3] - text_size[1]
    text_x = (img_size - text_w) // 2
    text_y = (img_size - text_h) // 2
    draw.text((text_x, text_y-img_size/2+10), character, font=font, fill=0)
    # Convert image to binary (black and white)
    img = img.resize((grid_size, grid_size), Image.Resampling.LANCZOS)  # Resize to fit 20x20 grid
    img_array = np.array(img) < 128  # Convert to boolean mask (True for black pixels)
    img.show()

    # Get pixel positions of the character
    y_coords, x_coords = np.where(img_array)
    points = np.array(list(zip(x_coords, y_coords)))

    # Ensure exactly 20 evenly spaced dots using K-Means clustering
    if len(points) > num_dots:
        kmeans = KMeans(n_clusters=num_dots, random_state=42, n_init=10)
        kmeans.fit(points)
        points = kmeans.cluster_centers_  # Use cluster centers as the chosen points
        points = np.round(points).astype(int)  # Convert to integer grid positions
    elif len(points) < num_dots:
        # If fewer than 20 points, duplicate existing points
        extra_needed = num_dots - len(points)
        points = np.concatenate((points, points[:extra_needed]))  # Repeat some points

    return points.tolist()

def plot_character(character):
    """Plot exactly 20 evenly spaced dots forming the character on a 20x20 grid."""
    grid_size = 50
    points = get_character_pixels(character, grid_size, num_dots=200)

    fig, ax = plt.subplots(figsize=(6, 6))

    for (x, y) in points:
        ax.scatter(x, grid_size - y - 1, color='blue', s=100)  # Flip y-axis for correct orientation

    ax.set_xticks(range(grid_size))
    ax.set_yticks(range(grid_size))
    ax.set_xlim(-0.5, grid_size - 0.5)
    ax.set_ylim(-0.5, grid_size - 0.5)
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_title(f"Character: {character}")

    plt.show()

# Example usage:
character = input("Enter a character (A-Z): ").upper()
plot_character(character)