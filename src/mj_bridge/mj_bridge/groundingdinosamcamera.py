import transformers
print(f"Transformers path: {transformers.__file__}")
print(f"Transformers version: {transformers.__version__}")
import cv2
import torch
import numpy as np
import time
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple, Union
from PIL import Image
from transformers import AutoModelForMaskGeneration, AutoProcessor, pipeline

# ================= 数据结构定义 (Data Structures) =================

@dataclass
class BoundingBox:
    xmin: int
    ymin: int
    xmax: int
    ymax: int

    @property
    def xyxy(self) -> List[float]:
        return [self.xmin, self.ymin, self.xmax, self.ymax]

@dataclass
class DetectionResult:
    score: float
    label: str
    box: BoundingBox
    mask: Optional[np.array] = None

    @classmethod
    def from_dict(cls, detection_dict: Dict) -> 'DetectionResult':
        return cls(score=detection_dict['score'],
                   label=detection_dict['label'],
                   box=BoundingBox(xmin=detection_dict['box']['xmin'],
                                   ymin=detection_dict['box']['ymin'],
                                   xmax=detection_dict['box']['xmax'],
                                   ymax=detection_dict['box']['ymax']))

# ================= 辅助函数 (Helper Functions) =================
# ================= NMS (去重) 辅助函数 =================
# ==========================================
# 📐 新增功能：根据形状自动区分长/短积木
# ==========================================

def identify_brick_type(detection: DetectionResult) -> str:
    """
    通过计算边界框的长宽比，判断是长积木还是短积木。
    """
    xmin, ymin, xmax, ymax = detection.box.xyxy
    
    width = xmax - xmin
    height = ymax - ymin
    
    # 避免除以零
    if height == 0 or width == 0:
        return "Unknown"

    # 计算长宽比 (长边 / 短边)
    # 结果总是 >= 1.0
    # 如果是正方形，结果接近 1.0
    # 如果是长方形，结果会显著大于 1.0
    ratio = max(width, height) / min(width, height)
    
    # --- 阈值判断逻辑 ---
    # 2x2 积木通常接近 1.0，稍微有点误差可能到 1.3
    # 2x4 积木通常在 1.5 到 2.0 之间
    
    if ratio < 1.4:
        return "Short (2x2)"  # 正方形
    else:
        return "Long (2x4)"   # 长方形

# ==========================================

def calculate_iou(box1, box2):
    """计算两个框的重叠度 (Intersection over Union)"""
    # 计算重叠区域的坐标
    x1 = max(box1.xmin, box2.xmin)
    y1 = max(box1.ymin, box2.ymin)
    x2 = min(box1.xmax, box2.xmax)
    y2 = min(box1.ymax, box2.ymax)

    # 计算重叠面积
    intersection = max(0, x2 - x1) * max(0, y2 - y1)

    # 计算两个框各自的面积
    box1_area = (box1.xmax - box1.xmin) * (box1.ymax - box1.ymin)
    box2_area = (box2.xmax - box2.xmin) * (box2.ymax - box2.ymin)

    # 计算并集面积
    union = box1_area + box2_area - intersection

    return intersection / union if union > 0 else 0

def filter_double_detections(detections: List[DetectionResult], iou_threshold: float = 0.5) -> List[DetectionResult]:
    """
    NMS 算法：如果两个框重叠超过 50%，只保留分数高的那个。
    这样同一个物体就不会有两个标签了。
    """
    if not detections:
        return []
    
    # 1. 按分数从高到低排序 (分数高的排前面)
    detections = sorted(detections, key=lambda x: x.score, reverse=True)
    
    keep = []
    
    for current in detections:
        is_duplicate = False
        # 检查当前框是否和我们已经保留的框重叠
        for kept in keep:
            if calculate_iou(current.box, kept.box) > iou_threshold:
                is_duplicate = True # 找到了重叠且分数更高的大哥，当前这个小弟就不要了
                break
        
        if not is_duplicate:
            keep.append(current)
            
    return keep

def get_boxes(results: List[DetectionResult]) -> List[List[List[float]]]:
    boxes = []
    for result in results:
        xyxy = result.box.xyxy
        boxes.append(xyxy)
    return [boxes]

def mask_to_polygon(mask: np.ndarray) -> List[List[int]]:
    # 寻找轮廓
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []
    # 找到最大的轮廓
    largest_contour = max(contours, key=cv2.contourArea)
    polygon = largest_contour.reshape(-1, 2).tolist()
    return polygon

def polygon_to_mask(polygon: List[Tuple[int, int]], image_shape: Tuple[int, int]) -> np.ndarray:
    mask = np.zeros(image_shape, dtype=np.uint8)
    pts = np.array(polygon, dtype=np.int32)
    cv2.fillPoly(mask, [pts], color=(1,))
    return mask

def refine_masks(masks: torch.BoolTensor, polygon_refinement: bool = False) -> List[np.ndarray]:
    masks = masks.cpu().float()
    masks = masks.permute(0, 2, 3, 1)
    masks = masks.mean(axis=-1)
    masks = (masks > 0).int()
    masks = masks.numpy().astype(np.uint8)
    masks = list(masks)

    if polygon_refinement:
        for idx, mask in enumerate(masks):
            shape = mask.shape
            polygon = mask_to_polygon(mask)
            if polygon:
                mask = polygon_to_mask(polygon, shape)
                masks[idx] = mask
    return masks

def annotate_video_frame(frame_bgr: np.ndarray, detection_results: List[DetectionResult]) -> np.ndarray:
    """
    在 OpenCV 的 BGR 图像上直接绘制框和掩码
    """
    # 复制一份以免修改原图
    image_cv2 = frame_bgr.copy()

    for detection in detection_results:
        label = detection.label
        score = detection.score
        box = detection.box
        mask = detection.mask

        # 随机颜色
        color = np.random.randint(0, 256, size=3).tolist()

        # 1. 画框
        cv2.rectangle(image_cv2, (box.xmin, box.ymin), (box.xmax, box.ymax), color, 2)
        
        # 2. 写标签
        text = f'{label}: {score:.2f}'
        cv2.putText(image_cv2, text, (box.xmin, box.ymin - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 3. 画掩码 (Mask)
        if mask is not None:
            # 创建半透明遮罩
            mask_uint8 = (mask * 255).astype(np.uint8)
            
            # 方法 A: 仅画轮廓 (速度快，清晰)
            contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image_cv2, contours, -1, color, 2)
            
            # 方法 B: 颜色填充 (可选，这里用叠加方式)
            colored_mask = np.zeros_like(image_cv2, dtype=np.uint8)
            colored_mask[mask > 0] = color
            # 将掩码与原图混合
            image_cv2 = cv2.addWeighted(image_cv2, 1.0, colored_mask, 0.4, 0)

    return image_cv2

# ================= 核心逻辑 (Core Logic) =================

def detect_video(
    image: Image.Image,
    detector,
    labels: List[str],
    threshold: float = 0.3
) -> List[DetectionResult]:
    # 处理标签格式 (加上 . 以提高 DINO 准确率)
    labels = [label if label.endswith(".") else label + "." for label in labels]
    
    results = detector(image, candidate_labels=labels, threshold=threshold)
    return [DetectionResult.from_dict(result) for result in results]

def segment_video(
        image: Image.Image,
        detection_results: List[DetectionResult],
        segmentator,
        processor,
        polygon_refinement: bool = False,
        device: str = "cpu"
) -> List[DetectionResult]:
    
    boxes = get_boxes(detection_results)
    
    # 预处理输入
    inputs = processor(images=image, input_boxes=boxes, return_tensors="pt")

    # [关键修复] 针对 MPS (Mac) 设备的类型错误修复
    if 'input_boxes' in inputs and inputs['input_boxes'].dtype == torch.float64:
        inputs['input_boxes'] = inputs['input_boxes'].to(torch.float32)

    # 移动到设备 (GPU/CPU)
    inputs = inputs.to(device)

    # 推理
    with torch.no_grad():
        outputs = segmentator(**inputs)

    # 后处理掩码
    masks = processor.post_process_masks(
        masks=outputs.pred_masks,
        original_sizes=inputs.original_sizes,
        reshaped_input_sizes=inputs.reshaped_input_sizes
    )[0]

    masks = refine_masks(masks, polygon_refinement)

    # 将掩码分配回结果对象
    for detection_result, mask in zip(detection_results, masks):
        detection_result.mask = mask

    return detection_results

# ================= 主程序 (Main) =================

if __name__ == "__main__":
    
    # --- 1. 配置参数 (Settings) ---
    WEBCAM_ID = 4              # 🚨 如果打不开，尝试改成 0, 2, 4, 6
    PROCESS_SIZE = (640, 480)  # 降低分辨率以提高 FPS
    CONF_THRESHOLD = 0.35      # 稍微调高门槛，减少误识别
    
    # 提示词：建议用描述性的词
    LABELS = ["black brick.", "white brick.","blue brick.","red brick."] 

    DETECTOR_ID = "IDEA-Research/grounding-dino-tiny"
    SEGMENTER_ID = "facebook/sam-vit-base"

    # --- 2. 设备检测 ---
    if torch.cuda.is_available():
        DEVICE = "cuda"
        print(">>> 正在使用 NVIDIA GPU (CUDA) 🚀")
    elif torch.backends.mps.is_available():
        DEVICE = "mps"

        print(">>> 正在使用 Apple Silicon (MPS)")
    else:
        DEVICE = "cpu"
        print(">>> 警告：正在使用 CPU，速度较慢")

    # --- 3. 加载模型 ---
    print(">>> 正在加载模型 (请稍候)...")
    try:
        detector_pipeline = pipeline(model=DETECTOR_ID, task="zero-shot-object-detection", device=DEVICE)
        sam_model = AutoModelForMaskGeneration.from_pretrained(SEGMENTER_ID).to(DEVICE)
        sam_processor = AutoProcessor.from_pretrained(SEGMENTER_ID)
        print(">>> ✅ 模型加载成功！")
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        exit()

    # --- 4. 打开摄像头 ---
    cap = cv2.VideoCapture(WEBCAM_ID)
    if not cap.isOpened():
        print(f"❌ 错误：无法打开摄像头 ID {WEBCAM_ID}")
        print("   建议: 运行 'ls /dev/video*' 查看可用设备，或尝试更改 WEBCAM_ID")
        exit()

    print(">>> 系统运行中... 按 'q' 键退出")

    while cap.isOpened():
        start_time = time.time()
        ret, frame = cap.read()
        if not ret: break

        # A. 预处理
        frame = cv2.flip(frame, 1)
        frame_resized = cv2.resize(frame, PROCESS_SIZE)
        image_pil = Image.fromarray(cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB))

        # B. 目标检测 (Grounding DINO)
        detections = detect_video(image_pil, detector_pipeline, LABELS, CONF_THRESHOLD)

        # 🔥 C. 关键步骤：NMS 去重 (只保留重叠物体中分数最高的一个)
        if detections:
            detections = filter_double_detections(detections, iou_threshold=0.5)

        # D. 实例分割 (SAM)
        if detections:
            detections = segment_video(image_pil, detections, sam_model, sam_processor, device=DEVICE)
            annotated_frame = annotate_video_frame(frame_resized, detections)
        else:
            annotated_frame = frame_resized

        # E. 显示 FPS
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # F. 显示结果
        cv2.imshow("Lego Sorter AI", annotated_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()