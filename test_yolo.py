from ultralytics import YOLO

class YoloEval:
    def __init__(self, model_path):
        self.model = YOLO(model_path)

    def eval(self, image_path, conf = 0.25, img_save = False):
        results = self.model.predict(source=image_path, conf=conf)
        for i, result in enumerate(results):
            if img_save:
                result.save(filename=f"results_{i}.jpg")

if __name__ == "__main__":
    yolo_eval = YoloEval("path/to/model_weight/best.pt")
    yolo_eval.eval(["sample_test.jpg"], img_save=True)