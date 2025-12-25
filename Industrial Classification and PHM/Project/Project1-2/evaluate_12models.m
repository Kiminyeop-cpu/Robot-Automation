%% ============================================================
%  12개 모델 자동 성능 평가 (ACC / AUC / 저장)
%% ============================================================

clear; clc;

load("trainTestData.mat", "testData");

datasets = {
    "fan_dbm6", "fan_db0", "fan_dbp6", ...
    "pump_dbm6", "pump_db0", "pump_dbp6", ...
    "slider_dbm6", "slider_db0", "slider_dbp6", ...
    "valve_dbm6", "valve_db0", "valve_dbp6"
};

results = table('Size',[12 4], ...
    'VariableTypes', {'string','double','double','string'}, ...
    'VariableNames', {'Dataset','Accuracy','AUC','ModelFile'});

idx = 1;

for d = datasets
    ds = d{1};

    fprintf("\n🔍 Evaluating model: %s\n", ds);

    % Load model
    modelFile = sprintf("net_%s.mat", ds);
    S = load(modelFile);
    net = S.net;

    % Load test data
    Xtest = testData.(ds).X;
    Ytest = categorical(testData.(ds).Y);

    % Prediction
    YPred = classify(net, Xtest);
    acc = mean(YPred == Ytest);

    % AUC
    scores = predict(net, Xtest);
    [~,~,~,AUC] = perfcurve(Ytest, scores(:,2), "1");

    % Save result row
    results(idx,:) = {ds, acc*100, AUC, modelFile};
    idx = idx + 1;
end

disp("=======================================");
disp("📊 12개 모델 성능 요약");
disp(results);
disp("=======================================");

% 저장
assignin("base","results",results);
save("model_performance_summary_fixed.mat","results");

writetable(results,"model_performance_summary.xlsx");
