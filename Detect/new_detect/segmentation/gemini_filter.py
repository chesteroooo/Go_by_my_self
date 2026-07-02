import os
import shutil
import time
from PIL import Image
from google import genai
from google.genai import types
from google.genai.errors import APIError

# ==================== 付費極速版設定區 ====================
INPUT_DIR = "./data1"                 # 你拍的一大堆原始照片資料夾
OUTPUT_CLEAR = "./filtered_data/clear"        # 可標註
OUTPUT_MINOR = "./filtered_data/minor_issues" # 有小瑕疵
OUTPUT_BAD = "./filtered_data/bad_data"       # 不可用（糊的、黑的）

# ⚡ 付費版 RPM 高達 1500，每張圖中間只需要留 0.2 秒給硬碟和網路緩衝即可
DELAY_TIME = 0.2 
# ========================================================

# 建立分類目標資料夾
for folder in [OUTPUT_CLEAR, OUTPUT_MINOR, OUTPUT_BAD]:
    os.makedirs(folder, exist_ok=True)

# 初始化 Gemini Client (會自動去抓系統環境變數裡的 GEMINI_API_KEY)
try:
    client = genai.Client()
except Exception as e:
    print(f"❌ 儀表板初始化失敗，請確認是否有設定環境變數 GEMINI_API_KEY。錯誤原因: {e}")
    exit(1)

# 嚴格的 Prompt 指令，逼 AI 只能回答關鍵字，方便程式進行 If-Else 分類
PROMPT = """
你是一個硬體電腦視覺資料集的篩選專家。請幫我評估這張由自走車相機拍攝的室外測試照片。
請嚴格根據以下標準進行分類：
- 如果畫面清晰程度夠，可清楚判斷路面以及其他物件等等，可當作語意分割的標註素材，直接輸出一個詞："可標註"
- 如果有些微晃動模糊、逆光、過曝，打致還能辨識，勉強可當作語意分割標註素材，輸出一個詞："有小瑕疵"
- 如果照片嚴重糊掉、動態殘影、鏡頭被擋住、畫面一片模糊不清，完全無法當作語意分割標註素材，輸出一個詞："不可用"

注意：你只能輸出上述三個詞之一（不需要引號），絕對不要提供任何多餘的解釋、標點符號或客套話。
"""

print("🚀 Gemini 測試資料批次篩選系統 [🔥 付費極速全開版] 已啟動...")
print(f"📂 正在掃描資料夾：{INPUT_DIR}")

# 開始跑迴圈掃描資料夾內的所有圖片
image_extensions = ('.png', '.jpg', '.jpeg')
all_files = sorted([f for f in os.listdir(INPUT_DIR) if f.lower().endswith(image_extensions)])
total_files = len(all_files)

print(f"📊 偵測到共有 {total_files} 張圖片待處理。")

# 紀錄總開始時間
start_time = time.time()

for index, img_name in enumerate(all_files, start=1):
    img_path = os.path.join(INPUT_DIR, img_name)
    print(f"⚡ [{index}/{total_files}] 正在極速分析: {img_name} ... ", end="", flush=True)
    
    while True:
        try:
            # 讀取圖片
            img = Image.open(img_path)
            
            # 付費版依然建議保留縮圖，檔案小、傳輸快，效率最高
            img.thumbnail((500, 500))
            
            # 呼叫 Gemini 模型
            response = client.models.generate_content(
                model='gemini-2.5-flash',
                contents=[img, PROMPT]
            )
            
            # 清理 AI 回傳的字串
            ai_decision = response.text.strip()
            
            # 根據 AI 的判斷移動檔案
            if "可標註" in ai_decision:
                shutil.move(img_path, os.path.join(OUTPUT_CLEAR, img_name))
                print("✅ [可標註]")
            elif "有小瑕疵" in ai_decision:
                shutil.move(img_path, os.path.join(OUTPUT_MINOR, img_name))
                print("⚠️ [有小瑕疵]")
            elif "不可用" in ai_decision:
                shutil.move(img_path, os.path.join(OUTPUT_BAD, img_name))
                print("❌ [不可用]")
            else:
                shutil.move(img_path, os.path.join(OUTPUT_MINOR, img_name))
                print(f"❓ [格式異常] (AI回應: {ai_decision})")
            break # 成功處理完畢，跳出 While 迴圈
            
        except APIError as ae:
            # 付費版若偶爾遇到網路或雲端小波動，原地等 3 秒直接重試
            print(f"\n☁️ 遭遇伺服器微幅波動: {ae}。原地等待 3 秒後自動重試該張...")
            time.sleep(3)
            continue
        except Exception as e:
            print(f" 💥 本地損壞錯誤: {e}。跳過此張。")
            try:
                shutil.move(img_path, os.path.join(OUTPUT_MINOR, img_name))
            except:
                pass
            break
            
    # 極短的暫停，留給硬碟搬移檔案與網路緩衝
    time.sleep(DELAY_TIME)

end_time = time.time()
print(f"\n🎉 任務完成了！極速洗完 {total_files} 張照片，總耗時: {round((end_time - start_time)/60, 2)} 分鐘。")