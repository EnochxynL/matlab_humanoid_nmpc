classdef DiagnosticWriter < handle
    %DIAGNOSTICWRITER MATLAB owns diagnostic files; native code returns values.
    properties (Access = private)
        folder
        files
    end
    methods
        function obj = DiagnosticWriter(folder)
            obj.folder = folder;
            if ~isfolder(folder), mkdir(folder); end
            obj.files = containers.Map('KeyType','char','ValueType','double');
            obj.writeText('reference_authority.txt',sprintf('SIMULINK_REFERENCE\n'));
        end
        function append(obj,batch)
            for k = 1:numel(batch.tables)
                table = batch.tables(k);
                assert(~isempty(regexp(table.name,'^[a-z_]+$','once')), ...
                    'G1:DiagnosticSchema','Invalid diagnostic table name');
                if ~isKey(obj.files,table.name)
                    fid = fopen(fullfile(obj.folder,[table.name '.csv']),'w','n','UTF-8');
                    assert(fid>=0,'G1:DiagnosticFile','Cannot open diagnostic table');
                    obj.files(table.name) = fid;
                    fprintf(fid,'%s\n',strjoin(table.columns,','));
                end
                values = table.values;
                if isempty(values), continue; end
                formats = repmat({'%.17g'},1,size(values,2));
                textColumns = cellfun(@ischar,values(1,:));
                formats(textColumns) = {'%s'};
                for j = find(textColumns)
                    values(:,j) = cellfun(@(s)['"' strrep(s,'"','""') '"'], ...
                        values(:,j),'UniformOutput',false);
                end
                format = [strjoin(formats,',') '\n'];
                % One formatted write per table/batch, in row order.
                argumentsInOrder = reshape(values.',1,[]);
                fprintf(obj.files(table.name),format,argumentsInOrder{:});
            end
            if ~isempty(batch.configurationJson)
                obj.writeText('loaded_configuration.json',batch.configurationJson);
            end
            if ~isempty(batch.loadedLibraries)
                obj.writeText('loaded_process_maps.txt',batch.loadedLibraries);
            end
            if ~isempty(batch.snapshot)
                file = fullfile(obj.folder,'structural_snapshot.bin');
                fid = fopen(file,'w');
                assert(fid>=0,'G1:DiagnosticFile','Cannot open structural snapshot');
                cleanup = onCleanup(@()fclose(fid));
                fwrite(fid,batch.snapshot,'double');
            end
        end
        function delete(obj)
            if isempty(obj.files), return; end
            identifiers = values(obj.files);
            for k=1:numel(identifiers), fclose(identifiers{k}); end
            remove(obj.files,keys(obj.files));
        end
        function sample(obj,tick,rawTime,canonicalTime,result)
            columns = [{'control_tick','raw_time','canonical_time'}, ...
                arrayfun(@(j)sprintf('value%d',j),0:117,'UniformOutput',false)];
            values = num2cell([tick rawTime canonicalTime reshape(result,1,[])]);
            table = struct('name','planner_samples','columns',{columns},'values',{values});
            batch = struct('tables',table,'snapshot',[], ...
                'configurationJson','','loadedLibraries','');
            obj.append(batch);
        end
        function timeIdentity(obj,tick,rawTime,canonicalTime)
            columns = {'raw_backend_time','control_tick','canonical_control_time', ...
                'raw_minus_canonical','session_id','state_sequence'};
            values = num2cell([rawTime tick canonicalTime rawTime-canonicalTime 1 tick]);
            table = struct('name','time_identity','columns',{columns},'values',{values});
            batch = struct('tables',table,'snapshot',[], ...
                'configurationJson','','loadedLibraries','');
            obj.append(batch);
        end
        function flush(obj)
            names = keys(obj.files);
            for k=1:numel(names)
                name = names{k};
                fid = obj.files(name);
                remove(obj.files,name);
                fclose(fid);
                fid = fopen(fullfile(obj.folder,[name '.csv']),'a','n','UTF-8');
                assert(fid>=0,'G1:DiagnosticFile','Cannot reopen diagnostic table');
                obj.files(name) = fid;
            end
        end
    end
    methods (Access = private)
        function writeText(obj,name,text)
            fid = fopen(fullfile(obj.folder,name),'w','n','UTF-8');
            assert(fid>=0,'G1:DiagnosticFile','Cannot open diagnostic metadata');
            cleanup = onCleanup(@()fclose(fid));
            fprintf(fid,'%s',text);
        end
    end
end
