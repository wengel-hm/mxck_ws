from ultralytics import YOLO

model = YOLO('yolov8n-seg.yaml')  # build a new model from YAML
model = YOLO('yolov8n-seg.pt')  # Transfer the weights from a pretrained model

project = 'path_to_result_folder' # Define destination directory for all results
name = 'epochs-' # Define subdirectory for this specific training
results = model.train(data='path_to_yaml_file', # instert path to yaml file
                      project=project,
                      name=name,
                      fliplr = 0) #fliplr = 0 important for lane detection, as otherwise right and left line can't be distinguished