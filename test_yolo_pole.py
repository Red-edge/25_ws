#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import json
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np
from inference_sdk import InferenceHTTPClient


# ========= 你需要改的配置 =========
IMG_DIR = "/Users/rededge/Documents/25_ws/traffic/raw_images"  # 改成你的图片目录
WORKSPACE_NAME = "25ws"
WORKFLOW_ID = "find-dim-traffic-lights-red-traffic-lights-green-traffic-lights-and-stop-signs"
API_URL = "https://serverless.roboflow.com"

# 建议在 shell 里设置：export ROBOFLOW_API_KEY="xxxx"
API_KEY = os.environ.get("ROBOFLOW_API_KEY", "").strip()

# 是否保存可视化结果（按 s 触发保存）
SAVE_DIR = os.path.join(IMG_DIR, "_rf_vis")


# ========= 一些工具函数 =========
def list_images(img_dir: str) -> List[str]:
    paths = sorted(
        glob.glob(os.path.join(img_dir, "*.jpg"))
        + glob.glob(os.path.join(img_dir, "*.jpeg"))
        + glob.glob(os.path.join(img_dir, "*.png"))
    )
    return paths


def encode_jpg_bytes(bgr: np.ndarray, quality: int = 85) -> bytes:
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    return buf.tobytes()


def safe_get_predictions(workflow_result: Any) -> List[Dict[str, Any]]:
    """
    Roboflow workflow 返回结构会因 workflow 设计不同而不同。
    这个函数尽量从结果里“捞出” predictions 列表。
    你如果想严格适配，可把 result 打印出来看真实结构。
    """
    # result 可能是 list / dict
    if isinstance(workflow_result, list):
        # 常见：list 里每个 step 一个 dict
        preds = []
        for step in workflow_result:
            preds.extend(safe_get_predictions(step))
        return preds

    if isinstance(workflow_result, dict):
        # 常见键："predictions" / "outputs" / "output"
        if "predictions" in workflow_result and isinstance(workflow_result["predictions"], list):
            return workflow_result["predictions"]

        for k in ["outputs", "output", "result", "results", "data"]:
            if k in workflow_result:
                return safe_get_predictions(workflow_result[k])

    return []


def pred_to_xyxy(pred: Dict[str, Any]) -> Tuple[int, int, int, int]:
    """
    尽量兼容常见 Roboflow 格式：
    - (x, y, width, height) 以中心点为 x/y
    - 或 (x1,y1,x2,y2)
    """
    if all(k in pred for k in ["x1", "y1", "x2", "y2"]):
        return int(pred["x1"]), int(pred["y1"]), int(pred["x2"]), int(pred["y2"])

    if all(k in pred for k in ["x", "y", "width", "height"]):
        x = float(pred["x"])
        y = float(pred["y"])
        w = float(pred["width"])
        h = float(pred["height"])
        x1 = int(x - w / 2.0)
        y1 = int(y - h / 2.0)
        x2 = int(x + w / 2.0)
        y2 = int(y + h / 2.0)
        return x1, y1, x2, y2

    # 兜底：不给框
    return 0, 0, 0, 0


def pred_label(pred: Dict[str, Any]) -> str:
    name = str(pred.get("class", pred.get("label", pred.get("name", "obj"))))
    conf = pred.get("confidence", pred.get("conf", None))
    if conf is None:
        return name
    try:
        return f"{name} {float(conf):.2f}"
    except Exception:
        return name


def draw_predictions(bgr: np.ndarray, preds: List[Dict[str, Any]]) -> np.ndarray:
    h, w = bgr.shape[:2]
    vis = bgr.copy()

    for pred in preds:
        x1, y1, x2, y2 = pred_to_xyxy(pred)
        # clamp
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w - 1, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h - 1, y2))
        if x2 <= x1 or y2 <= y1:
            continue

        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

        txt = pred_label(pred)
        cv2.putText(vis, txt, (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 中心点和 bias（可选）
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        cv2.circle(vis, (cx, cy), 4, (0, 255, 0), -1)
        u = (cx - w / 2.0) / (w / 2.0)
        v = (cy - h / 2.0) / (h / 2.0)
        cv2.putText(vis, f"bias u={u:+.2f}, v={v:+.2f}",
                    (x1, min(h - 10, y2 + 22)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return vis


# ========= 主程序 =========
def main():
    if not API_KEY:
        raise RuntimeError(
            "ROBOFLOW_API_KEY not set. Run:\n"
            "  export ROBOFLOW_API_KEY='YOUR_KEY'\n"
            "then re-run."
        )

    paths = list_images(IMG_DIR)
    if not paths:
        raise RuntimeError(f"No images found in: {IMG_DIR}")

    os.makedirs(SAVE_DIR, exist_ok=True)

    client = InferenceHTTPClient(api_url="https://serverless.roboflow.com", api_key="MszC0oJSRNaKp1pWaCsv")

    idx = 0
    cache_last = {}  # 可以缓存每张图的结果

    print("Controls: n(next) p(prev) s(save vis) r(re-run) j(print json) q(quit)")

    while True:
        p = paths[idx]
        bgr = cv2.imread(p)
        if bgr is None:
            print("skip unreadable:", p)
            idx = min(idx + 1, len(paths) - 1)
            continue

        # 推理：优先用缓存（避免你反复上一张下一张重复扣额度/耗时）
        if p in cache_last:
            result = cache_last[p]
        else:
            jpg_bytes = encode_jpg_bytes(bgr, quality=85)
            result = client.run_workflow(
                workspace_name=WORKSPACE_NAME,
                workflow_id=WORKFLOW_ID,
                images={"image": p},   # <-- 直接传图片路径
                use_cache=True
            )
            cache_last[p] = result

        preds = safe_get_predictions(result)
        vis = draw_predictions(bgr, preds)

        # UI 文本
        h, w = vis.shape[:2]
        cv2.putText(vis, f"{idx+1}/{len(paths)}  {os.path.basename(p)}  preds={len(preds)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(vis, "n/p:next/prev  s:save  r:rerun  j:json  q:quit",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("roboflow_workflow_viewer", vis)
        key = cv2.waitKey(0) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("n"):
            idx = min(idx + 1, len(paths) - 1)
        elif key == ord("p"):
            idx = max(idx - 1, 0)
        elif key == ord("s"):
            out_path = os.path.join(SAVE_DIR, os.path.basename(p))
            cv2.imwrite(out_path, vis)
            print("saved:", out_path)
        elif key == ord("r"):
            result = client.run_workflow(
            workspace_name=WORKSPACE_NAME,
            workflow_id=WORKFLOW_ID,
            images={"image": p},   # <-- 这里也改成 p
            use_cache=True
            )
            cache_last[p] = result
            print("re-run done:", os.path.basename(p))
        elif key == ord("j"):
            print(json.dumps(result, ensure_ascii=False, indent=2))
        else:
            # 其他键：默认下一张
            idx = min(idx + 1, len(paths) - 1)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
