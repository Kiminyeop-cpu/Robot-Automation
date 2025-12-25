%% ============================================================
%   FULL AUTO TRAINING — 12개 모델 자동 학습 + 진행률 표시
%% ============================================================

clear; clc;

load("trainTestData.mat", "trainData", "testData");

options = trainingOptions("adam", ...
    "MaxEpochs", 60, ...
    "MiniBatchSize", 32, ...
    "InitialLearnRate", 1e-3, ...
    "Shuffle", "every-epoch", ...
    "Verbose", true, ...
    "Plots", "training-progress");

inputSize = [128 311 2];

datasets = {
    "fan_dbm6", "fan_db0", "fan_dbp6", ...
    "pump_dbm6", "pump_db0", "pump_dbp6", ...
    "slider_dbm6", "slider_db0", "slider_dbp6", ...
    "valve_dbm6", "valve_db0", "valve_dbp6"
};

totalModels = length(datasets);

%% 전체 타이머 시작
tic;
fprintf("\n🔵 총 %d개 모델 학습 시작...\n", totalModels);

for i = 1:totalModels
    ds = datasets{i};

    %% 🔥 진행률 표시
    fprintf("\n===============================================\n");
    fprintf(" [%d / %d] 학습 시작 → %s\n", i, totalModels, ds);
    fprintf(" 전체 진행률: %.1f%%\n", (i-1)/totalModels*100);
    fprintf("===============================================\n");

    modelStart = tic;   % 모델별 타이머 시작

    %% 데이터 로드
    Xtrain = trainData.(ds).X;
    Ytrain = categorical(trainData.(ds).Y);

    Xtest  = testData.(ds).X;
    Ytest  = categorical(testData.(ds).Y);

    %% 모델 생성
    lgraph = model_MobileNetV3(inputSize, 2);

    %% 학습
    net = trainNetwork(Xtrain, Ytrain, lgraph, options);

    %% 모델 저장
    saveName = sprintf("net_%s.mat", ds);
    save(saveName, "net");
    fprintf("💾 모델 저장됨 → %s\n", saveName);

    %% 모델별 소요시간 표시
    elapsed = toc(modelStart);
    fprintf("⏱ %s 학습 완료 — %.1f초 걸림\n", ds, elapsed);

    %% Accuracy / AUC
    YPred = classify(net, Xtest);
    acc = mean(YPred == Ytest);

    scores = predict(net, Xtest);
    [~, ~, ~, AUC] = perfcurve(Ytest, scores(:,2), '1');

    fprintf(" ✔ Accuracy = %.2f%%\n", acc * 100);
    fprintf(" ✔ AUC      = %.3f\n", AUC);
end

%% 전체 시간 표시
totalTime = toc;
fprintf("\n=====================================================\n");
fprintf(" 🎉 모든 %d개 모델 학습 완료!\n", totalModels);
fprintf(" ⏱ 총 소요시간: %.1f초 (%.1f분)\n", totalTime, totalTime/60);
fprintf("=====================================================\n");
