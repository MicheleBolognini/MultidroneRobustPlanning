function vfprintf(verbose, varargin)
% VFPRINTF Display output optionally depending on the level of verbosity.
%
% VFPRINTF(TF, ARGS) passes the arguments ARGS to the built-in MATLAB
% command |fprintf| if TF is logical true. If TF is logical false, VFPRINTF
% does nothing.

assert(islogical(verbose) && isscalar(verbose),...
    'utils:InvalidVerbose',...
    'VERBOSE must be logical true or false');

if verbose
    fprintf(varargin{:});
end
