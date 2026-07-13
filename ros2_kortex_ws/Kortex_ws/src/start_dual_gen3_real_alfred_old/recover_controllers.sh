#!/bin/bash
# ⚠️ WARNING: 自動化機械手臂控制器復位腳本 (防彈跳安全防護版)
# 此腳本用於在 Kinova Gen3 手臂急停/手動拖動復原後，安全地重啟控制器以避免重新啟動整個 Driver。

echo "========== 開始復原右手控制器 (Right Arm Controller Recovery) =========="

echo "停止控制器中 (若已停止則自動忽略錯誤)..."
ros2 control set_controller_state -c /right/controller_manager joint_trajectory_controller inactive >/dev/null 2>&1
ros2 control set_controller_state -c /right/controller_manager robotiq_gripper_controller inactive >/dev/null 2>&1

echo "重啟並啟用控制器中..."
success=true

ros2 control set_controller_state -c /right/controller_manager joint_trajectory_controller active
if [ $? -ne 0 ]; then
  success=false
fi

ros2 control set_controller_state -c /right/controller_manager robotiq_gripper_controller active
if [ $? -ne 0 ]; then
  success=false
fi

if [ "$success" = true ]; then
  echo "✅ 右手控制器復原成功！"
else
  echo "❌ 復原失敗，請確認手臂目前是否已經亮綠燈，且 right/controller_manager 正在運行。"
fi
