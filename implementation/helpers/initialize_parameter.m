function value = initialize_parameter(structure, field_name, default_value)
% Return value of a given field from a given structure. If the field 
% doesn't exist, return the default value.
%
% This function helps to deal with the initialization of parameters.
%
% Input: structure
%        field_name [string]
%        default_value
% Output: value
%
if ~isfield(structure,field_name) % if field doesn't exist, return default value
    value = default_value;
else
    value = structure.(field_name);
end
end