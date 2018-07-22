function positive_spectrum = rfft(time_domain_signal)
double_sided_spectrum = fft(time_domain_signal);
nr_of_bins = length(double_sided_spectrum);
if mod(nr_of_bins,2) == 1
    fprintf('attention: odd nr of bins');
end
positive_spectrum = double_sided_spectrum(1:round(nr_of_bins/2)+1);
