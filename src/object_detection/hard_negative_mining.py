import os
from ultralytics import YOLO
import cv2

# Initialize YOLO model
model_inst_seg = YOLO('path_to_weights')  # Update with the path to your YOLOv8 weights

# Define directories
img_folder = 'path_to_images'  # Update with the path to your images folder
label_folder = 'path_to_labels'  # Update with the path to your labels folder
inference_folder_path = 'path_to_inference_results'  # Path where the predicted images are stored

# Get list of image files
img_files = [f for f in os.listdir(img_folder) if os.path.isfile(os.path.join(img_folder, f))]

# Process images and generate predictions
for index in range(len(img_files)):
    img_path = os.path.join(img_folder, img_files[index])
    results = model_inst_seg.predict(img_path, conf=0.2, save=True)  # Perform inference

    # Save labels in corresponding .txt files
    for result in results:
        label_path = os.path.join(label_folder, img_files[index].replace('.jpg', '.txt'))
        result.save_txt(label_path)

# Initialize lists for managing saved images and labels
save_img = []
label_files = [f for f in os.listdir(label_folder) if os.path.isfile(os.path.join(label_folder, f))]
label_path = [os.path.join(label_folder, f) for f in label_files]

# Display and review inference results
index = 0
while True:
    inference_img_files = [f for f in os.listdir(inference_folder_path) if os.path.isfile(os.path.join(inference_folder_path, f))]
    inference_img_path = os.path.join(inference_folder_path, inference_img_files[index])
    img = cv2.imread(inference_img_path)

    # Show image in fullscreen
    cv2.namedWindow('Image Review', cv2.WINDOW_NORMAL)
    cv2.setWindowProperty('Image Review', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow('Image Review', img)

    # Handle user input
    key = cv2.waitKeyEx(0)
    if key == 2490368:  # Arrow up key
        cv2.destroyAllWindows()
        save_img.append(inference_img_files[index])
        os.remove(inference_img_path)

        if index == len(inference_img_files) - 1:
            index = 0
        else:
            index += 1

    elif key == 2555904:  # Arrow right key
        cv2.destroyAllWindows()
        index = (index + 1) % len(inference_img_files)

    elif key == 2424832:  # Arrow left key
        cv2.destroyAllWindows()
        index = (index - 1) % len(inference_img_files)

    elif key == 13:  # Enter key
        cv2.destroyAllWindows()
        save_img_set = set(os.path.join(img_folder, img) for img in save_img)
        save_label_set = set(os.path.join(label_folder, img.replace('.jpg', '.txt')) for img in save_img)

        # Remove unselected images and labels
        for img in os.listdir(img_folder):
            img_path = os.path.join(img_folder, img)
            if img_path not in save_img_set:
                os.remove(img_path)
                print(f'{img_path} has been deleted.')

        for label in label_files:
            label_path = os.path.join(label_folder, label)
            if label_path not in save_label_set:
                os.remove(label_path)
                print(f'{label_path} has been deleted.')

        # Clean up inference folder
        for img in os.listdir(inference_folder_path):
            os.remove(os.path.join(inference_folder_path, img))
        os.rmdir(inference_folder_path)
        break

    else:
        print("Invalid input. Use arrow keys to navigate or Enter to finish.")
        cv2.destroyAllWindows()
