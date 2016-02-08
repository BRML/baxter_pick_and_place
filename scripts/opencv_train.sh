opencv_traincascade \
-data /home/baxter/ros_ws/src/baxter_pick_and_place/data/sdd/ \
-vec /home/baxter/ros_ws/src/baxter_pick_and_place/data/sdd/bin_info.dat \
-bg /home/baxter/ros_ws/src/baxter_pick_and_place/data/sdd/bg.txt \
# numPos <= (#Vec - numNeg)/(1 + (numStages - 1)(1 - minHitRate)
-numPos 120 \
-numNeg 75 \
-numStages 14 \
-precalcValBufSize 1024 \
-precalcIdxBufSize 1014 \
-acceptanceRationBreakValue 10e-5 \
-featureType HAAR \
-w 240 \
-h 320 \
-bt GAB \
-minHitRate 0.999 \
-weightTrimRate 0.95 \
-maxDepth 1 \
-mode ALL
