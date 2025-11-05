from PIL import Image, ImageDraw
import os

def draw_bridge_and_close_image(img, grayscale=True):

     # Ensure image is in grayscale
    if img.mode != "L":
        img = img.convert("L")

    # Get dimensions
    width, height = img.size

    # Compute band position and thickness
    line_thickness = int(height * 0.10)   # 10% of height
    y_top = int(height * 0.25)            # start at 25%
    y_bottom = y_top + line_thickness

    draw = ImageDraw.Draw(img)

    # Draw a black horizontal band (fill=0)
    draw.rectangle(
        [(0, y_top), (width, y_bottom)],
        fill=0
    )

    # Draw white 1-pixel borders at top and bottom (fill=0)
    fill_color = 0
    px_thick = 1
    draw.rectangle([(0, 0), (width, px_thick)], fill=fill_color)
    draw.rectangle([(0, height - px_thick), (width, height)], fill=fill_color)

    return img


def main():
    # setup dir structure
    script_dir = os.path.dirname(os.path.abspath(__file__))
    OUT_DIR = os.path.join(script_dir, 'out_imgs', 'tif_img_editor')
    os.makedirs(OUT_DIR, exist_ok=True)

    # Load image
    IMAGE_FILENAME = 'geo_wildcat_mosaic_mask.tif'
    IMAGE_FILE = os.path.join(script_dir, 'imgs', IMAGE_FILENAME)


    # Generate output filename
    base, ext = os.path.splitext(os.path.basename(IMAGE_FILENAME))
    output_name = os.path.join(OUT_DIR, f"{base}_with_band{ext}")

    img = draw_bridge_and_close_image(img = Image.open(IMAGE_FILE))
    # Save and show
    img.save(output_name)
    img.show()

    print(f"Saved image as: {output_name}")


if __name__ == '__main__':
    main()
