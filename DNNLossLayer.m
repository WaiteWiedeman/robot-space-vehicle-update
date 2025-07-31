classdef DNNLossLayer < nnet.layer.RegressionLayer ...
        & nnet.layer.Acceleratable
    % Example custom regression layer with mean-absolute-error loss.
    
    methods
        function layer = DNNLossLayer(name)
            % layer = weightedLossLayer(name) creates a
            % mean-sqaure-error regression layer and specifies the layer
            % name.
            % Set layer name.
            layer.Name = name;
            % Set layer description.
            layer.Description = 'Weighted Loss Layer';
        end
        
        function loss = forwardLoss(layer, Y, T)
            % Inputs:
            %         layer - Output layer
            %         Y     – Predictions made by network
            %         T     – Training targets
            % data loss: compute the difference between target and predicted values
            dataLoss = mape(Y, T,"all"); %mse(Y,T);
            % final loss, combining data loss and physics loss
            loss = dataLoss; % (1.0-trainParams.alpha-trainParams.beta)*dataLoss + trainParams.alpha*physicLoss + trainParams.beta*endEffloss;
        end

        function dLdY = backwardLoss(layer,Y,T)
            % (Optional) Backward propagate the derivative of the loss 
            % function.
            %
            % Inputs:
            %         layer - Output layer
            %         Y     – Predictions made by network
            %         T     – Training targets
            %
            % Output:
            %         dLdY  - Derivative of the loss with respect to the 
            %                 predictions Y        
            dLdY = 2*(Y-T)/numel(T);
        end
    end
end