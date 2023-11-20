function [label_x, label_y] = axes_labels_invariants(datatype, parameterization)

% Pick the correct labels for the X- and Y-axis based on the given data type
if strcmp(parameterization,'arclength_normalized')

    label_x = '$\xi$ [-]';

    if strcmp(datatype,'pose')
        label_y = {'$a$ [rad/-]','$\omega_\kappa$ [rad/-]','$\omega_\tau$ [rad/-]','$b$ [m/-]','$v_b$ [m/-]','$v_t$ [m/-]'};
    elseif strcmp(datatype,'wrench')
        label_y = {'$a$ [N]','$\omega_\kappa$ [rad/-]','$\omega_\tau$ [rad/-]','$b$ [Nm]','$v_b$ [m/-]','$v_t$ [m/-]'};
    elseif strcmp(datatype,'position')
    elseif strcmp(datatype,'orientation')
    elseif strcmp(datatype,'force')
    elseif strcmp(datatype,'moment')
    else % generic labels
        label_y = {'$i_1$','$i_2$','$i_3$','$i_4$','$i_5$','$i_6$'};
    end

elseif strcmp(parameterization,'time_based')

    label_x = '$t$ [s]';

    if strcmp(datatype,'pose')
        label_y = {'$a$ [rad/s]','$\omega_\kappa$ [rad/s]','$\omega_\tau$ [rad/s]','$b$ [m/s]','$v_b$ [m/s]','$v_t$ [m/s]'};
    elseif strcmp(datatype,'wrench')
        label_y = {'$a$ [N]','$\omega_\kappa$ [rad/s]','$\omega_\tau$ [rad/s]','$b$ [Nm]','$v_b$ [m/s]','$v_t$ [m/s]'};
    elseif strcmp(datatype,'position')
    elseif strcmp(datatype,'orientation')
    elseif strcmp(datatype,'force')
    elseif strcmp(datatype,'moment')
    else % generic labels
        label_y = {'$i_1$','$i_2$','$i_3$','$i_4$','$i_5$','$i_6$'};
    end

elseif strcmp(parameterization,'arclength')

    label_x = '$s$ [m]';

    if strcmp(datatype,'pose')
        label_y = {'$a$ [rad/m]','$\omega_\kappa$ [rad/m]','$\omega_\tau$ [rad/m]','$b$ [m/m]','$v_b$ [m/m]','$v_t$ [m/m]'};
    elseif strcmp(datatype,'wrench')
        label_y = {'$a$ [N]','$\omega_\kappa$ [rad/m]','$\omega_\tau$ [rad/m]','$b$ [Nm]','$v_b$ [m/m]','$v_t$ [m/m]'};
    elseif strcmp(datatype,'position')
    elseif strcmp(datatype,'orientation')
    elseif strcmp(datatype,'force')
    elseif strcmp(datatype,'moment')
    else % generic labels
        label_y = {'$i_1$','$i_2$','$i_3$','$i_4$','$i_5$','$i_6$'};
    end

else % generic labels
    label_x = '$\xi$';
    label_y = {'$i_1$','$i_2$','$i_3$','$i_4$','$i_5$','$i_6$'};
end

end