import os
from ultralytics import YOLO
import cv2

# Initialize YOLO model and define directories
model_inst_seg = YOLO('path_to_weights')
img_folder = 'path_to_images'
label_folder = 'path_to_labels'
inference_folder_path = 'path_to_inference_results' # usually /runs/segment/predict

# Get list of image files and initialize img_path list
img_files = [f for f in os.listdir(img_folder) if os.path.isfile(os.path.join(img_folder, f))]
img_path = [None] * len(img_files)

# Process images and generate predictions   
for index in range(len(img_files)):
    
    img_path[index] = os.path.join(img_folder, img_files[index])
    results = model_inst_seg.predict(img_path[index], conf=0.2, save=True)
    print(img_files[index])

    # Save labels in corresponding .txt files
    for result in results:
        convert_jpg_to_txt = img_files[index][:-4]
        label_path = os.path.join(f'{label_folder}', f'{convert_jpg_to_txt}.txt')
        text = result.save_txt(label_path)

# Initialize lists for managing saved images and labels        
label_files = [f for f in os.listdir(label_folder) if os.path.isfile(os.path.join(label_folder, f))]
label_path = [None] * len(label_files)
for i in range(len(label_files)):
    label_path[i] = os.path.join(label_folder, label_files[i])
save_img = []

# Display and review inference results
index = 0
while True:
    inference_img_files = [f for f in os.listdir(inference_folder_path) if os.path.isfile(os.path.join(inference_folder_path, f))]
    inference_img_path = os.path.join(inference_folder_path, inference_img_files[index])
    img = cv2.imread(inference_img_path)
    window_title = f'Bild: {inference_img_files[index]}'
        
    # show image
    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(window_title, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow(window_title, img)

    # handle user input   
    key = cv2.waitKeyEx(0) 
        
    if key == 2490368:  #up key
        cv2.destroyAllWindows()
        save_img.append(inference_img_files[index])

        if index == len(inference_img_files)-1:
            i = index
            index = 0
            os.remove(os.path.join(inference_folder_path, inference_img_files[i])) 
        else: os.remove(inference_img_path)
           
    elif key == 2555904:  #right key
        cv2.destroyAllWindows()
        if index == len(inference_img_files)-1:
            index = 0
        else: index += 1

    elif key == 2424832: #left key
        cv2.destroyAllWindows()
        if index == 0:
            index = len(inference_img_files)-1
        else: index -= 1

    elif key == 13: #enter key
        cv2.destroyAllWindows()
        save_img_path = []
        save_label = []
        save_label_path = []

        for i in range(len(save_img)):
            convert_jpg_to_txt = save_img[i][:-4] + '.txt'
            save_label.append(convert_jpg_to_txt)
            save_label_path.append(os.path.join(label_folder, save_label[i]))

        for i in range(len(save_img)):
            save_img_path.append(os.path.join(img_folder, save_img[i]))

        # Remove unselected images
        save_img_set = set(save_img_path)
        for img in img_path:
         if img not in save_img_set:
                os.remove(img)
                print(f'{img} has been deleted.')

        # Remove unselected labels
        save_label_set = set(save_label_path)
        for label in label_path:
            if label not in save_label_set:
                os.remove(label)
                print(f'{label} has been deleted.')

        # remove rest of the images in inference_folder_path
        inference_img_files = [f for f in os.listdir(inference_folder_path) if os.path.isfile(os.path.join(inference_folder_path, f))]
        for index in range(len(inference_img_files)):
            os.remove(os.path.join(inference_folder_path, inference_img_files[index]))

        # remove temporary inference_folder_path, so next time they will be stored in the same folder    
        os.rmdir(inference_folder_path) 
        break
        
    else: 
        print("Invalid input. Use arrow keys to navigate or Enter to finish.")
        cv2.destroyAllWindows() 
         
