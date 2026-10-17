function validate_adapters(root, allowRoundoff)
%VALIDATE_ADAPTERS Frozen conversions and reference transport; no planner.
if nargin < 2, allowRoundoff = false; end
folder = fullfile(root,'build','windows');
addpath(fullfile(folder,'adapters'),fullfile(folder,'probes'));
cleanup = onCleanup(@()rmpath(fullfile(folder,'adapters'),fullfile(folder,'probes')));
assert(mingw_probe() == 81);
golden = load(fullfile(root,'build','observation_hybrid','conversion_values.mat'));
result = struct('passed',false,'release',version,'compiler','MinGW GCC 8.1.0 POSIX SEH', ...
    'physical_simulation',false,'conversion_cases_checked',0, ...
    'roundoff_continuation_authorized',allowRoundoff,'strict_bitwise_pass',true, ...
    'different_doubles',0,'max_abs',0);
for j = 1:size(golden.raw,1)
    raw = golden.raw(j,:)';
    euler = g1_observation_mex('quaternion_to_euler',raw(5:8));
    world = g1_observation_mex('rotate_linear_velocity',raw(5:8),raw(9:11));
    rates = g1_observation_mex('angular_velocity_to_euler_rates',euler,raw(12:14));
    actual = [euler;world;rates];
    assert(all(isfinite(actual)),'Nonfinite conversion output');
    expected = golden.expected(j,[4:6,30:35])';
    bitsActual = typecast(actual,'uint64');
    bitsExpected = typecast(expected,'uint64');
    different = bitsActual ~= bitsExpected;
    result.conversion_cases_checked = j;
    result.max_abs = max(result.max_abs,max(abs(actual-expected)));
    result.different_doubles = result.different_doubles + nnz(different);
    if any(different)
      result.strict_bitwise_pass = false;
      if ~isfield(result,'first_case')
        result.first_case = j;
        result.first_component = find(different,1);
        result.max_abs_this_case = max(abs(actual-expected));
        result.different_doubles_this_case = nnz(different);
        if ~isfile(fullfile(folder,'first_conversion_difference.mat'))
          save(fullfile(folder,'first_conversion_difference.mat'), ...
              'raw','actual','expected','bitsActual','bitsExpected','result');
        end
      end
      if ~allowRoundoff
        writeResult(folder,result,'adapters-acceptance.json');
        error('G1:WindowsConversionDifference', ...
            'Strict gate failed at case %d, component %d, maxabs %.17g. Stopped.', ...
            j,result.first_component,result.max_abs_this_case);
      end
    end
end
source = jsondecode(fileread(fullfile(root,'data','g1_source.json')));
if iscell(source.joints)
  names = cellfun(@(joint)joint.name,source.joints,'UniformOutput',false);
else
  names = {source.joints.name};
end
[present,indices] = ismember(source.mpc_joint_names,names);
assert(all(present));
indices = double(indices(:)');
for j = 1:2401
  identity = g1_observation_extract_mex(golden.raw(j,:)',golden.expected(j,:)',indices);
  assert(isequal(identity,[double(j-1);double(j-1)*.002;golden.raw(j,1)]));
end
result.observation_ticks_checked = 2401;
raw = golden.raw(1,:)'; state = golden.expected(1,:)';
bad = raw; bad(4) = .2; reject(@()g1_observation_extract_mex(bad,state,indices));
bad = raw; bad(1) = .001; reject(@()g1_observation_extract_mex(bad,state,indices));
bad = raw; bad(5:8) = 0; reject(@()g1_observation_extract_mex(bad,state,indices));
bad = raw; bad(9) = NaN; reject(@()g1_observation_extract_mex(bad,state,indices));
bad = state; bad(7) = bad(7)+1; reject(@()g1_observation_extract_mex(raw,bad,indices));
bad = indices; bad(2) = bad(1); reject(@()g1_observation_extract_mex(raw,state,bad));
result.observation_negative_cases = 6;
f = load(fullfile(root,'build','core_extraction','fixture.mat'));
for j = 1:size(f.ref,1)
    actual = g1_reference_extract_mex(f.ref(j,:)');
    assert(isequal(typecast(actual(:),'uint64'),typecast(f.ref(j,:)','uint64')));
end

function reject(callback)
rejected = false;
try
  ignored = callback(); %#ok<NASGU>
catch failure
  rejected = strcmp(failure.identifier,'G1:ObservationExtractionMex');
end
assert(rejected,'Invalid observation accepted');
end
result.passed = true;
result.reference_cases_checked = size(f.ref,1);
if allowRoundoff
  writeResult(folder,result,'adapters-authorized-continuation.json');
else
  writeResult(folder,result,'adapters-acceptance.json');
end
disp(result);
end

function writeResult(folder,result,filename)
fid = fopen(fullfile(folder,filename),'w');
assert(fid >= 0);
cleanup = onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));
end
