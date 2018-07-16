function [ vEuler ] = quat2vec( q )
        vEuler(1) = atan2(2*(q(4)*q(1) + q(2)*q(3)) , (1-2*(q(1)*q(1) + q(2)*q(2))));   
        vEuler(2) = asin(2*(q(4)*q(2) - q(3)*q(1)));                                   
        vEuler(3) = atan2(2*(q(4)*q(3) + q(1)*q(2)) , (1-2*(q(2)*q(2) + q(3)*q(3))));  


end

