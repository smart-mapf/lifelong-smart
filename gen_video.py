import cv2
import os
import fire


def gen_video(frame_dir, output_mp4="output.mp4", fps=30):

    # Get sorted list of frame files
    frames = sorted([f for f in os.listdir(frame_dir) if f.endswith(".png")])

    assert len(frames) > 0, "No frames found!"

    # Read first frame to get size
    first_frame = cv2.imread(os.path.join(frame_dir, frames[0]))
    height, width, _ = first_frame.shape

    # Define video writer (H.264-compatible)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video = cv2.VideoWriter(output_mp4, fourcc, fps, (width, height))

    for fname in frames:
        path = os.path.join(frame_dir, fname)
        img = cv2.imread(path)
        if img is None:
            raise RuntimeError(f"Failed to read {path}")
        video.write(img)

    video.release()
    print(f"Saved video to {output_mp4}")


if __name__ == "__main__":
    fire.Fire(gen_video)
