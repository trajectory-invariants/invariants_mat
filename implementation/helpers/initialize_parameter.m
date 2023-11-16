function value = initialize_parameter(structure, field_name, default_value)
% Returns field value in a given structure. 
% If the field doesn't exist, return the default value.
%
% This function helps to deal with the initialization of parameters that may or may not have been specified.
% It is similar in functionality to Python where you can provide a default value to a function.
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