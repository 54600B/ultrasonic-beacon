opengl software;

carriers = [];
nr_of_us_signals = 3+4;
nr_of_samples = 2048; %adc samples
sender_oversampling = 2; %dac samples = adc samples * sender_oversampling
sample_rate = 160*1000;

bin_width = sample_rate / nr_of_samples
bin_spacing = 32;
first_bin = 289;
evaluation_bins = 64;

lowest_bin = zeros([1,nr_of_us_signals]);
center_bin = zeros([1,nr_of_us_signals]);
for i=1:nr_of_us_signals
    center_bin(i) = first_bin+(i-1)*bin_spacing;
    carriers(i) = center_bin(i)*bin_width;
    
    lowest_bin(i) = center_bin(i) - (evaluation_bins/2);
end


fileID = fopen('constants.h','w');
fprintf(fileID,'#ifndef CONSTANTS_H_\n');
fprintf(fileID,'#define CONSTANTS_H_\n');
fprintf(fileID,'//7 channel below 40kHz \n');
fprintf(fileID,'const uint32_t sender[%u][%u] = {',nr_of_us_signals,nr_of_samples*sender_oversampling);

spectra = zeros([nr_of_us_signals*2, nr_of_samples]); %//= { bin0re; bin0im; bin1re; bin1im; ... ; bin1023re; bin1023im}
figure;
spectra_fig = axes;
figure;
small_spect_plot = axes;
figure;
signals_fig = axes;

speaker_signals = zeros([7, nr_of_samples*2]);

for us_signal_nr = 1:nr_of_us_signals
    fprintf(fileID,'\n{');
    [wf_sinc_i,wf_sinc_q] = generate_sinc_modulated_pulse( carriers(us_signal_nr), 12 );
    
    offset = 2048.0 * 12.0/15.0;
    amplitude = 2048.0 * 14.0/15.0;
    for i = 1:nr_of_samples*2
        wf_sinc_i(i) =  round(offset+(amplitude/2)*wf_sinc_i(i)) ;
        wf_sinc_q(i) =  round(offset+(amplitude/2)*wf_sinc_q(i)) ;
    end
    
    plot(signals_fig, wf_sinc_i, 'DisplayName', strcat(num2str(us_signal_nr),'i'));
    hold(signals_fig, 'on');
    plot(signals_fig, wf_sinc_q, 'DisplayName', strcat(num2str(us_signal_nr),'q'));
    
    spectrum_i = rfft(wf_sinc_i(1:2:end)); 
    spectrum_q = rfft(wf_sinc_q(1:2:end));
    
    for i=1:(nr_of_samples/2)
        spectra((us_signal_nr*2)-1,(i*2)-1) = real(spectrum_i(i));
        spectra((us_signal_nr*2)-1,(i*2)  ) = imag(spectrum_i(i));
        
        spectra((us_signal_nr*2)  ,(i*2)-1) = real(spectrum_q(i));
        spectra((us_signal_nr*2)  ,(i*2)  ) = imag(spectrum_q(i));
    end
    
    freq_axis = 0:(80000/(nr_of_samples/2)):80000;
    plot(spectra_fig, freq_axis,abs( (spectrum_i*1i) + spectrum_q), 'DisplayName', strcat(num2str(us_signal_nr),' Carrier=',num2str(carriers(us_signal_nr)),'Hz'));
    hold(spectra_fig, 'on');
    %plot(freq_axis,abs(spectrum_q));
    %plot(wf_sinc_i);
    
    offset = 2048.0 * 12.0/15.0;
    amplitude = 2048.0 * 14.0/15.0;
    for i = 1:(nr_of_samples*2)-1
        fprintf(fileID, '%4u, ', wf_sinc_i(i) );
        if mod(i,20)==0
            fprintf(fileID, '\n');
        end
    end
    fprintf(fileID, '%u }',wf_sinc_i(nr_of_samples*2) );
    
    if us_signal_nr ~= nr_of_us_signals
        fprintf(fileID,','); 
    end
    
    center_bin = first_bin+(us_signal_nr-1)*bin_spacing;
    lower_bin = center_bin - (evaluation_bins/2);
    upper_bin = lower_bin + evaluation_bins -1;
    plot(small_spect_plot, real(spectrum_i(lower_bin:upper_bin)),'DisplayName', strcat('real(',num2str(us_signal_nr),'i)'));
    hold(small_spect_plot, 'on');
    plot(small_spect_plot, imag(spectrum_i(lower_bin:upper_bin)),'DisplayName', strcat('imag(',num2str(us_signal_nr),'i)'));
    plot(small_spect_plot, real(spectrum_q(lower_bin:upper_bin)),'DisplayName', strcat('real(',num2str(us_signal_nr),'q)'));
    plot(small_spect_plot, imag(spectrum_q(lower_bin:upper_bin)),'DisplayName', strcat('imag(',num2str(us_signal_nr),'q)'));
    
    %save sender signals to mat-file
    speaker_signals(us_signal_nr,:) = wf_sinc_i;
    
end

save('speaker_signals','speaker_signals');

fprintf(fileID,'};\n\n');
legend(spectra_fig, 'show');
title(spectra_fig,'spectrum of senders');
xlabel(spectra_fig, 'Frequency in Hz');
ylabel(spectra_fig, 'Magnitude');
xlim(spectra_fig, [10000 80000]);
grid(spectra_fig);

legend(small_spect_plot, 'show');
title(small_spect_plot,'small spectrum bins');
xlabel(small_spect_plot, 'FFT Bin');
grid(small_spect_plot);

legend(signals_fig, 'show');
title(signals_fig, 'Signals');
xlabel(signals_fig, 'Sample');
ylabel(signals_fig, 'Amplitude');
xlim(signals_fig, [0 1500]);
grid(signals_fig);

fprintf(fileID, 'const uint32_t lowest_spectral_bin[%i] = {', nr_of_us_signals);
for spectrum_nr = 1:nr_of_us_signals
    fprintf(fileID, ' %f', lowest_bin(spectrum_nr));
    if spectrum_nr ~= nr_of_us_signals
        fprintf(fileID, ','); 
    end
end
fprintf(fileID, '};\n\n');


fprintf(fileID,'const float32_t spectra[%u][%u] = {',nr_of_us_signals*2,evaluation_bins*2);
for spectrum_nr = 1:(nr_of_us_signals*2)
    fprintf(fileID,'\n{');
    
    lowest_bin_of_small_spectrum = lowest_bin( floor( (spectrum_nr+1)/2 ) );
    for i=(lowest_bin_of_small_spectrum*2):((lowest_bin_of_small_spectrum+evaluation_bins-1)*2)-1
        fprintf(fileID, '%12.4f, ', spectra(spectrum_nr,i));
        if mod(i,10)==0
            fprintf(fileID, '// f=%.0f\n', 0.5*(160000.0/2048.0)*i);
        end
    end
    fprintf(fileID, '%u }', spectra(spectrum_nr, lowest_bin_of_small_spectrum+evaluation_bins-1));
    
    if spectrum_nr ~= (nr_of_us_signals*2)
        fprintf(fileID,',');
    end
end
fprintf(fileID,'};\n\n');

nr_of_gold_sequences = 3;

nr_of_gold_bits = 512; %actual gold codes have 511 bits , a zero is added to be compatible with the fft which requires a length of 2^n
gold_sequences = ... 
[0,0,0,0,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,0,0,1,1,1,0,1,0,0,0,1,0,0,1,0,1,0,0,0,1,1,1,0,0,1,0,0,1,0,1,1,0,0,1,0,0,1,1,0,0,0,1,0,0,1,1,1,0,0,1,1,1,0,0,1,1,1,1,0,0,0,0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,1,0,1,0,0,0,0,1,1,0,0,1,0,1,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,1,1,0,1,0,0,1,0,1,1,1,0,1,1,1,1,1,0,0,1,1,0,1,0,1,1,0,0,1,1,1,0,1,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,0,1,0,0,1,0,1,0,1,1,0,0,0,0,0,0,1,0,0,0,0,0,1,1,1,1,0,1,0,1,0,0,0,0,1,1,1,0,0,1,0,1,0,0,0,1,1,0,1,1,0,1,1,0,0,1,0,1,0,0,1,1,1,0,0,0,1,0,1,1,0,1,1,0,1,1,1,0,1,0,1,1,1,1,0,1,0,1,1,1,0,1,1,0,1,1,0,1,0,0,0,1,1,1,0,0,1,0,1,0,0,1,1,0,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,0,0,0,0,1,0,1,0,1,1,1,1,0,0,0,0,0,1,0,0,0,0,0,0,1,1,0,1,0,1,0,0,1,0,0,1,0,0,1,0,0,0,1,1,1,0,0,0,1,0,1,0,1,1,1,0,0,1,1,0,1,0,1,1,0,0,1,1,1,1,1,0,1,1,1,0,1,0,0,1,0,1,1,0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,1,0,1,0,1,0,1,1,1,0,1,1,1,0,0,0,0,1,0,0,0,0,1,1,1,1,0,0,1,1,1,0,0,1,1,1,0,0,1,0,0,0,1,1,0,0,1,0,0,1,1,0,1,0,0,1,0,0,1,1,1,0,0,0,1,0,1,0,0,1,0,0,0,1,0,1,1,1,0,0,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0; ...
 0,1,1,0,0,1,0,1,0,1,1,1,0,1,0,0,0,0,1,1,1,1,1,1,0,0,1,1,1,0,0,1,1,1,0,0,0,1,0,0,0,1,1,0,0,0,0,0,0,1,1,1,0,1,1,1,0,1,1,0,1,1,0,0,0,1,1,0,1,0,0,0,1,1,1,1,0,1,0,1,0,1,1,1,1,0,0,0,1,0,1,1,0,0,0,1,1,0,1,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,1,0,0,0,1,0,0,0,1,1,1,0,0,1,1,1,0,0,1,1,1,1,1,1,0,0,0,0,1,0,1,1,1,0,1,0,1,0,0,1,1,0,1,1,0,0,1,1,0,1,1,1,0,1,0,0,0,1,1,0,0,1,0,1,1,1,1,0,1,1,0,0,1,1,1,1,1,1,1,1,1,1,0,1,0,0,1,1,0,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1,0,0,1,0,1,1,0,1,1,1,0,0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,1,1,1,1,1,0,1,1,0,0,1,0,0,0,1,1,1,0,1,1,1,1,0,0,1,0,1,0,1,0,1,0,0,1,0,1,1,0,1,0,1,0,0,0,0,0,0,0,0,1,1,0,1,1,0,1,0,0,1,0,1,1,1,1,1,1,0,1,0,0,1,0,1,1,0,1,1,0,0,0,0,0,0,0,0,1,0,1,0,1,1,0,1,0,0,1,0,1,0,1,0,1,0,0,1,1,1,1,0,1,1,1,0,0,0,1,0,0,1,1,0,1,1,1,1,1,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,1,1,1,0,1,1,0,1,0,0,1,0,0,0,1,1,0,1,0,1,1,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,1,0,0,1,0,1,1,1,1,1,1,1,1,1,1,0,0,1,1,0,1,1,1,1,0,1,0,0,1,1,0,0,0,1,0,1,1,1,0,1,1,0,0,1,1,0; ...
 1,1,1,1,1,1,0,1,0,1,1,1,1,0,0,0,1,0,0,1,0,0,0,0,1,0,1,1,1,1,1,0,1,0,1,0,0,0,1,1,0,1,0,1,1,1,1,0,1,0,0,0,1,1,0,0,1,0,1,1,0,0,0,1,0,0,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,0,0,0,0,1,1,0,0,0,0,1,1,0,0,1,1,0,0,1,0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,1,0,0,1,1,0,0,1,1,0,0,0,0,1,1,0,0,0,0,1,1,1,1,0,0,1,0,1,0,0,0,1,1,0,1,1,0,0,1,0,0,0,1,1,0,1,0,0,1,1,0,0,0,1,0,1,1,1,1,0,1,0,1,1,0,0,0,1,0,1,0,1,1,1,1,1,0,1,0,0,0,0,1,0,0,1,0,0,0,1,1,1,1,0,1,0,1,1,1,1,1,1,1,0,0,1,0,0,1,0,1,1,0,1,1,1,1,1,0,1,0,1,1,0,0,1,1,1,0,0,1,1,1,0,0,0,0,0,1,0,0,0,1,1,1,1,0,1,1,1,0,1,0,1,0,0,1,0,0,1,0,0,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,1,1,0,0,1,1,0,0,0,0,0,1,1,0,1,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,1,1,1,1,1,1,1,0,1,0,0,1,1,0,1,1,0,1,0,1,0,0,1,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,1,0,0,1,0,1,0,1,1,0,1,1,0,0,1,0,1,1,1,1,1,1,1,1,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,1,1,0,1,1,0,0,0,0,0,1,1,0,0,1,1,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,0,0,1,0,0,1,0,0,1,0,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,0,0,0,0,1,1,1,0,0,1,1,1,0,0,1,1,0,1,0,1,1,1,1,1,0,1,1,0,1,0,0,1,0,0,1,0 ];

%<test rect wave>
% reduction_factor = 1
% 
% for gs=1:nr_of_gold_sequences
%     for i=1:nr_of_gold_bits      
%         if rem(i,reduction_factor*2) < reduction_factor
%             gold_sequences(gs,i) = 0;
%         else
%             gold_sequences(gs,i) = 1;
%         end
%     end 
% end
% gold_sequences
%</test rect wave>


for gs=1:nr_of_gold_sequences
    for i=1:180
        gold_sequences(gs,i) = 0;
        gold_sequences(gs,513-i) = 0;
    end
end

fprintf(fileID,'const uint8_t gold_seed[3][512] = {');
for gold_sequence_nr = 1:nr_of_gold_sequences 
    fprintf(fileID,'\n{');
    for bit = 1:511 
        fprintf(fileID, '%u, ', gold_sequences(gold_sequence_nr, bit));
        if mod(bit,32)==0
            fprintf(fileID, '\n');
        end
    end
    fprintf(fileID, '%u }', gold_sequences(gold_sequence_nr, 512));
    
    if (gold_sequence_nr ~= nr_of_gold_sequences)
       fprintf(fileID,',');
    end
end
fprintf(fileID,'};\n\n');

for gs=1:nr_of_gold_sequences
    for i=181:(512-180)
         gold_sequences(gs,i) = gold_sequences(gs,i) -0.5;
    end
end

gold_rec_spectum  = zeros([nr_of_gold_sequences, nr_of_samples/2]);
gold_rec_spectum_upper  = zeros([nr_of_gold_sequences, nr_of_samples/2]);
gold_rec_spectum_lower  = zeros([nr_of_gold_sequences, nr_of_samples/2]);

gold_rec_sequence = zeros([nr_of_gold_sequences, nr_of_samples]);
gold_rec_sequence_lower = zeros([nr_of_gold_sequences, nr_of_samples]);
gold_rec_sequence_upper = zeros([nr_of_gold_sequences, nr_of_samples]);

figure;
ir_spect_plot = axes;

strech_ratio = nr_of_samples/nr_of_gold_bits
for gs = 1:nr_of_gold_sequences
    for i=1:nr_of_samples
        gold_rec_sequence(gs,i) = gold_sequences( gs, floor((i-1)/strech_ratio)+1 ); 
    end
    
    %<chirp test>
    if gs == 3
        start = 0.353*nr_of_samples;
        stop = nr_of_samples-start;
        start_frequency = 30000;
        stop_frequency = 35000;
        phase = 0;
        for i = 1:nr_of_samples
            if i > start
                if i < stop                 
                    frequency = start_frequency + (stop_frequency - start_frequency) * ((i-start)/(stop-start));
                    phase = phase + ((frequency/sample_rate) * 2*pi);
                    gold_rec_sequence(gs, i) = sin(phase);
                end
            end
            
        end
    end
    %<chirp test>
    
    
    spectrum = rfft( gold_rec_sequence(gs,:) );
    
    
    freq_axis = 0:(80000/(nr_of_samples/2)):80000;
    plot(ir_spect_plot, freq_axis,abs(spectrum), 'DisplayName', strcat('IR_',num2str(gs)));
    hold(ir_spect_plot, 'on');
    
    for i=1:(nr_of_samples/2)
        gold_rec_spectrum(gs,(i*2)-1) = real(spectrum(i));
        gold_rec_spectrum(gs,(i*2)  ) = imag(spectrum(i));    
    end
    
    bins_to_zero = 1000/(160000/(nr_of_samples));
    for i=1:bins_to_zero
        gold_rec_spectrum(gs,(i*2)-1) = 0;
        gold_rec_spectrum(gs,(i*2)  ) = 0;
    end
end

title(ir_spect_plot, 'spectrum of IR-Signals');
xlabel(ir_spect_plot, 'frequency in Hz');
ylabel(ir_spect_plot, 'Amplitude');
legend(ir_spect_plot, 'show');
xlim(ir_spect_plot, [100, 80000]);
grid(ir_spect_plot);

%plot( gold_rec_sequence(1,:) );

fprintf(fileID,'const float32_t spectra_ir[%u][%u] = {',nr_of_gold_sequences,nr_of_samples);
for spectrum_nr = 1:(nr_of_gold_sequences)
    fprintf(fileID,'\n{');
    for i=1:nr_of_samples-1
        fprintf(fileID, '%f, ', gold_rec_spectrum(spectrum_nr,i));
        if mod(i,20)==0
            fprintf(fileID, '\n');
        end
    end
    fprintf(fileID, '%u }', gold_rec_spectrum(spectrum_nr, nr_of_samples));
    
    if spectrum_nr ~= (nr_of_gold_sequences)
        fprintf(fileID,',');
    end
end
fprintf(fileID,'};\n\n');

fprintf(fileID,'#endif\n');
%plot( abs(gold_rec_spectrum(gs,:)) );
fclose(fileID);


%plot auto correlation of gold codes
figure;
ir_autocorr_plot = axes;
for i = 1:nr_of_gold_sequences
   %dcfree = gold_sequences(i,:);
   dcfree = gold_rec_sequence(i,:);
   corr = xcorr(dcfree, dcfree);
   plot(ir_autocorr_plot, corr, 'DisplayName', strcat('IR_',num2str(i)));
   hold on;
end
title(ir_autocorr_plot, 'IR-Signals autocorrelation');
legend(ir_autocorr_plot, 'show');
xlabel(ir_autocorr_plot, 'shift in bins');
grid(ir_autocorr_plot);

figure;
%plot cross correlation of gold codes
for i = 1:nr_of_gold_sequences
   for j = 1:i
       if i~=j
           
           dcfree1 = gold_rec_sequence(i,:);
           dcfree2 = gold_rec_sequence(j,:);

           corr = xcorr(dcfree1, dcfree2);
           plot( corr, 'DisplayName', strcat('IR_',num2str(i)));
           hold on;
       end
   end
end
title('IR-Signals cross-correlation');
legend('show');
xlabel('shift in bins');
grid();

%<test correlation with small spectrum>
us_signal_nr = 1;
[wf_sinc_i,wf_sinc_q] = generate_sinc_modulated_pulse( carriers(us_signal_nr), 12 );

mic_signal = wf_sinc_i(1:2:end);
spectrum_i = rfft(wf_sinc_i(1:2:end));
spectrum_q = rfft(wf_sinc_q(1:2:end));

figure;
plot(abs(spectrum_i));

spectrum_i(280:300)
spectrum_q(280:300)

signal_rec_q = rifft(spectrum_q);
figure;
plot(signal_rec_q);

center_bin = first_bin+(us_signal_nr-1)*bin_spacing;
lower_bin = center_bin - (evaluation_bins/2)
upper_bin = lower_bin + evaluation_bins -1

spectrum_128_i = spectrum_i(lower_bin:upper_bin);
spectrum_128_q = spectrum_q(lower_bin:upper_bin);

spectrum_128_i = spectrum_128_i.*spectrum_128_i;
spectrum_128_q = spectrum_128_i.*spectrum_128_q;



figure;
plot(sqrt( (rifft(spectrum_128_i).^2) + (rifft(spectrum_128_q).^2)));



%</test correlation with small spectrum>

[wf_sinc_i1,wf_sinc_q1] = generate_sinc_modulated_pulse( carriers(1), 12 );
[wf_sinc_i2,wf_sinc_q2] = generate_sinc_modulated_pulse( carriers(2), 12 );

figure;
plot(xcorr(wf_sinc_i1, wf_sinc_i2));
hold on;
plot(xcorr(wf_sinc_i1, wf_sinc_i1));

s1 = fft(wf_sinc_i1);
s2 = fft(wf_sinc_i2);
g = s1.*conj(s2);
circ_corr = fftshift(ifft(g));

plot(real(circ_corr));
plot(imag(circ_corr));



