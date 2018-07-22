function [waveform_i,waveform_q] = generate_sinc_modulated_pulse( carrier, nr_bits )
    nr_of_samples = 4096;
    samplerate = 320*1000;
    timestep = 1/samplerate;
    duration = timestep * nr_of_samples;

    if ~exist('nr_bits','var')
      nr_bits = 28.0;
    end
    
    t_pulse = duration / nr_bits;
    n_sidelobes = 3;
    sinc_duration = t_pulse + (t_pulse/2)*(2*n_sidelobes);
    sinc_samples = sinc_duration / timestep;

    wf_sinc_i = zeros(1,nr_of_samples);
    wf_sinc_q = zeros(1,nr_of_samples);
    
    phase = -1*carrier*0.00002;
    
    for i = 1:sinc_samples
        time = i*timestep;
        sinc_param = (time/t_pulse)*2*pi - (1+n_sidelobes)*pi;
        envelope = sinc( sinc_param );

        wf_sinc_i(i) = envelope * sin(2*pi*carrier * time + phase);
        wf_sinc_q(i) = envelope * cos(2*pi*carrier * time + phase);
    end

    waveform_i = wf_sinc_i;
    waveform_q = wf_sinc_q;
end

