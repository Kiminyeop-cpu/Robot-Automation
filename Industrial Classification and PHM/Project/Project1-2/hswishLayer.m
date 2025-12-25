classdef hswishLayer < nnet.layer.Layer
    methods
        function layer = hswishLayer(name)
            layer.Name = name;
            layer.Description = "h-swish activation";
        end

        function Z = predict(layer, X)
            Z = X .* min(max(X + 3, 0), 6) / 6;
        end
    end
end
