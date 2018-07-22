function time_domain_signal = rifft(positive_spectrum)
nr_of_bins = length(positive_spectrum);  %including dc and nyquist frequency
double_sided_spectrum = positive_spectrum;%,zeros([1,nr_of_bins-2]);
for c=1:nr_of_bins-2
    double_sided_spectrum(nr_of_bins+c)=     real(double_sided_spectrum(nr_of_bins-c))...
                                          -(1i*imag(double_sided_spectrum(nr_of_bins-c)));
end
time_domain_signal = real(ifft(double_sided_spectrum));