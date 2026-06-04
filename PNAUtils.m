classdef PNAUtils

    methods (Static)
        function [sampleA] = pnaAcquisition(devPNA)
            try
                writeline(devPNA, '*OPC?');
                opc_response = readline(devPNA);  
                if str2double(strtrim(opc_response)) ~= 1
                    error("PNA is not ready (OPC)");
                end
        
                writeline(devPNA, 'SENS:SWE:MODE SING');
        
                writeline(devPNA, '*OPC?');
                opc_response = readline(devPNA);
                if str2double(strtrim(opc_response)) ~= 1
                    error("PNA is not ready (OPC)");
                end
        
                writeline(devPNA, 'CALC:PAR:SEL ''canal_A_ampli''');
                writeline(devPNA, 'CALC:DATA? FDAT');
                raw_ampli = readline(devPNA); 
                sampleAmpliA = str2double(strsplit(raw_ampli, ',')); 
        
                writeline(devPNA, 'CALC:PAR:SEL ''canal_A_fase''');
                writeline(devPNA, 'CALC:DATA? FDAT');
                raw_phase = readline(devPNA);
                samplePhaseA = str2double(strsplit(raw_phase, ','));
        
                sampleA = 10.^(sampleAmpliA/20) .* exp(1j * deg2rad(samplePhaseA));
        
            catch ME
                warning("Error in acquisition");
                disp(getReport(ME));
                sampleA = []; 
            end
        end
        
    end
end