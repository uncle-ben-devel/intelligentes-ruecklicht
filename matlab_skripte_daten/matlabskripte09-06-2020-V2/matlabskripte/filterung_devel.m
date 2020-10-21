clear;
%close all;
clc;

%% Einlesen von Dateien
% BNO055 V1
fID = fopen("Fahrt13_BNO055.txt", "r");
A = fscanf(fID, "%i %f %f %f %f", [5 inf]);
A = transpose(A);
time_data = A(:,2);
accel_data = A(:,3);
brems_data = A(:,4);
tacho_data = A(:,5);
clear A;
fclose(fID);

% BNO055 V2
% fID = fopen("Fahrt17_BNO055.txt", "r");
% A = fscanf(fID, "%i %f %f %f", [4 inf]);
% A = transpose(A);
% accel_data = A(:,2);
% brems_data = A(:,3);
% tacho_data = A(:,4);
% clear A;
% fclose(fID);

% ADXL345
% fID = fopen("Fahrt7_ADXL345.txt", "r");
% A = fscanf(fID, "%i %f %f %f %f %f", [6 inf]);
% A = transpose(A);
% accel_data = A(:,3);
% brems_data = A(:,5);
% tacho_data = A(:,6);
% clear A;
% fclose(fID);


%% Skript zur Darstellung und Filterung von digitalen Accelerometerdaten
%% Konstanten, Variablen
    vec_len = length(accel_data);         % Laenge der Datensaetze
    scale_factor = 4000;    % Werte kommen als int, wurden vorher von float mit scale_factor multipliziert fuer mehr Aufloesung. Fuer Rueckrechnung.
    achse_invertieren = -1; % Wenn accelerometer falschrum verbaut.
    f_sample = 100;         % in Hz
    t_achse_skala = 1000;   % fuer s
    
    accel_raw = accel_data / scale_factor * achse_invertieren;
    accel_raw = accel_raw - cumulative_avg(accel_raw);  % offset-Korrektur
    
    brems_vec = brems_data / 6;
    tacho_vec = tacho_data / scale_factor;
    %t_vec = time_data / t_achse_skala;
    t_vec = transpose((1:1:vec_len) / f_sample);% (0, 1/f_sample, vec_len);
    
%% Unterabtasten
    unterabtastrate = 2;
    offset = 0;
    accel_raw = unterabtasten(accel_raw, unterabtastrate, offset);
    brems_vec = unterabtasten(brems_vec, unterabtastrate, offset);
    tacho_vec = unterabtasten(tacho_vec, unterabtastrate, offset);
    t_vec = unterabtasten(t_vec, unterabtastrate, offset);
%% Clippen
%     accel_raw = clip_vals(accel_raw, 2);
    
%% Filtern mit verschiedenen Methoden
    filterbreite = 43;

    %% SMA - Simple moving average filter
    filter_mw0 = mittelwert(accel_raw,filterbreite);
    filter_mw1 = mittelwert(accel_raw,filterbreite*10);
    %% RMA - Rekursiver moving average filter
%     filter_mw_rec = mittelwert_rekursiv(accel_raw, filterbreite);
%     filter_mw_rec_k = mittelwert_rekursiv_kausal(accel_raw, filterbreite);
    %% Wavelet    
%     filter_wavelet = wdenoise(accel_raw);
    %% FIR - TP     
     y_filter_tp = FIR(accel_raw);
    %% MA mit eigener Gewichtung   
%     filter_ben_quad = mittelwert_benedikt_quad(accel_raw, filterbreite);
%     filter_ben_arctan = mittelwert_benedikt_arctan(accel_raw, filterbreite);
    %% MW zwischen peaks    
%     filter_peakmw = peak_mittelwert_filter(accel_raw, 5);
%     filter_peakmw = mittelwert(filter_peakmw,filterbreite);
    %% Slewratefilter    
    filter_max_delta = max_delta_filter(accel_raw, 0.08); % 200Hz: 0.08
    filter_max_delta_0 = mittelwert(filter_max_delta,0.1); % 200Hz: 10
    filter_max_delta_1 = mittelwert(filter_max_delta,10);
    %% PWM-Schwellwertfilter
%     filter_pwm = pwm_filter(accel_raw, mean(y_vec));
%     filter_pwm = mittelwert(filter_pwm,100);
    %% Extrapolation (funktioniert noch nicht, WIP)
%     extrapolated = extrapolate_lin(filter_max_delta, t_vec, 2);   
    %% Faltungsfilter mit Filterkernen, hier Gauss
%     windo = gausswin(10);
%     windo = windo / sum(windo, "all");
%     faltung_test0 = filter(windo,1,accel_raw);
%     
%     windo = gausswin(100);
%     windo = windo / sum(windo, "all");
%     faltung_test1 = filter(windo,1,accel_raw);
    %% ExpMA Filter
filter_exp_moving_avg0 = exp_moving_avg(accel_raw, 0.04);
filter_exp_moving_avg1 = exp_moving_avg(accel_raw, 0.07);
filter_exp_moving_avg2 = exp_moving_avg(accel_raw, 2/(filterbreite + 1));   % haeufiger Ansatz laut Wikipedia

%% Zusaetliche Daten nach der Filterung extrahieren
% 	filter_ableitung = ableitung_diskret(filter_max_delta_0, f_sample);
% 	filter_ableitung = mittelwert(ableitung,100);
%   filter_ableitung = ableitung_mit_delta(filter_exp_moving_avg0, 40, f_sample);
%  	filter_integration_diskret = integration_diskret(accel_raw, f_sample);
    
%% Statistik
%     hist_bin_num = 10^2;
%     y_stdev = std(accel_raw);
%     y_mean = median(accel_raw);
%     x_pd = linspace(-8, 8, 1000);
%     %y_normal_pd = 1/sqrt(2*pi*y_stdev)*exp(-1/2 * ((x_normal_pd - y_mean) / y_stdev).^2);
%     
%     % Laplace-Verteilung: https://en.wikipedia.org/wiki/Laplace_distribution
%     bb = sqrt(y_stdev/2);
%     y_laplace_pd = 1/(2*bb) * exp(-abs(x_pd-y_mean)/bb);
%     y_laplace_cd = cumsum(y_laplace_pd);
%     y_laplace_cd = y_laplace_cd / max(y_laplace_cd);
%     figure
%     hold on
%         histogram(accel_raw, "NumBins", hist_bin_num, "Normalization", "pdf");
%         plot(x_pd, y_laplace_pd, 'r');
%         title("Laplaceverteilung gegen geschaetzte WDF aus Histogram")
%         xlabel("Amplitudenwerte in m/s^2")
%         legend("Histogramm aus Messwerten", "Laplaceverteilung mit Kennwerten aus Messwerten")
%     hold off
%     
%     figure
%     hold on
%         histogram(accel_raw, "NumBins", hist_bin_num, "Normalization", "cdf");
%         plot(x_pd, y_laplace_cd, 'r');
%         title("Laplaceverteilung gegen geschaetzte kumulative Verteilungsdichte aus Histogram")
%         xlabel("Amplitudenwerte in m/s^2")
%         legend("Histogramm aus Messwerten", "Laplaceverteilung mit Kennwerten aus Messwerten")
%     hold off
%     
      stdev0 = moving_stdev(accel_raw, filterbreite);
     stdev1 = moving_stdev(accel_raw, filterbreite*10);
    cumulative_stdev0 = cumulative_stdev(accel_raw);

%% Spektrale Betrachtung
% [accel_amplitude, frq] = pwelch(accel_raw,[],[],[],f_sample, 'power');
% accel_amplitude = accel_amplitude / max(accel_amplitude);
% figure
% plot(frq, accel_amplitude)
% title("Spektrum der rohen Accelerometerwerte")
% xlabel("Frequenz in Hz")
% ylabel("Signalleistung linear, normiert auf 1")
% figure
% plot(frq, pow2db(accel_amplitude))
% title("Spektrum der rohen Accelerometerwerte")
% xlabel("Frequenz in Hz")
% ylabel("Signalleistung in dB, normiert auf 0dB")

%% Bremsen feststellen
brems_est_vec = wird_gebremst(filter_exp_moving_avg0, -0.5, filter_exp_moving_avg0, -0.5);
brems_est_vec = hysterese(brems_est_vec, 0.05, 0.15, f_sample);

%% Plotten
    plot_compare(accel_raw, brems_est_vec, filter_exp_moving_avg0, tacho_vec, brems_vec, t_vec);
    
%% Funktionen
function [] = plot_all(x_vec, y_vec, z_vec, tacho_vec, brems_vec, t_vec)
       hold on
            plx = plot(t_vec,x_vec,  'r');
            ply = plot(t_vec,y_vec,  'g');
            plz = plot(t_vec,z_vec,  'b');
            plta = plot(t_vec,tacho_vec, 'k');
            plbr = plot(t_vec,brems_vec, 'm');
       hold off
       
       plx.Color(4) = 0.1;
       ply.Color(4) = 0.1;
       plz.Color(4) = 0.05;
       
       title("Messwerte Testfahrt")
       
       legend(  "X-Achse Beschleunigung in m/s^2", ...
                "Y-Achse Beschleunigung in m/s^2", ...
                "Z-Achse Beschleunigung in m/s^2", ...
                "Tachometer Beschleunigung in m/s^2", ...
                "Bremshebel betaetigt ja:high, nein:low" ...
                );
            
        xlabel("Zeit in s")
        ylabel("Beschleunigung in m/s^2")
end

function [] = plot_compare(raw, filter0, filter1, tacho_vec, brems_vec, t_vec)
       figure
       hold on
            plx = plot(t_vec,raw,  'r');
            ply = plot(t_vec,filter0,  'g');
            plz = plot(t_vec,filter1,  'b');
            plta = plot(t_vec,tacho_vec, 'k');
            plbr = plot(t_vec,brems_vec, 'm.-');
       hold off
       
       plx.Color(4) = 0.1;
       %ply.Color(4) = 0.02;
       %plz.Color(4) = 0.02;
       
       title("Messwerte Testfahrt")
       
       legend(  "RAW Beschleunigung in m/s^2", ...
                "Gefilterte Beschleunigung 0 in m/s^2", ...
                "Gefilterte Beschleunigung 1 in m/s^2", ...
                "Tachometer Beschleunigung in m/s^2", ...
                "Bremshebel betaetigt ja:high, nein:low" ...
                );
            
        xlabel("Zeit in s")
        ylabel("Beschleunigung in m/s^2")
end

% holt nur jeden "unterabtastrate"sten Wert aus dem Vektor, mit einer Verschiebung um "offset modulo unterabtastrate".
function [y_unterabtast] = unterabtasten(y_vec, unterabtastrate, offset)
    y_len = length(y_vec);
    y_unterabtast = zeros(y_len / unterabtastrate,1);
    
    for ii=1:1:(y_len / unterabtastrate)
        if (ii*unterabtastrate + mod(offset,unterabtastrate)) < y_len
            y_unterabtast(ii) = y_vec(ii*unterabtastrate + mod(offset,unterabtastrate));
        else
            y_unterabtast(ii) = y_vec(ii*unterabtastrate);
        end
    end
end

function [y] = mittelwert(x, mw_breite)
    x_len = length(x);
    y = zeros(x_len,1);
    
    for ii=mw_breite:x_len
        for kk=0:mw_breite-1
            y(ii,1) = y(ii,1) + x(ii-kk,1);
        end
    end
    
    y = y / mw_breite;
end

% rekursiver Mittelwertsfilter. Breite muss ungerade sein. Problem: nicht
% kausal! nutzt Werte aus der Zukunft.
function [y] = mittelwert_rekursiv(x, mw_breite)
    % wenn mw_breite gerade, dann um 1 erhoehen.
    filterbreite = mw_breite + not(mod(mw_breite,2));
    x_len=length(x);
    y = zeros(x_len, 1);
    
    % rekursiver Mittelwertsfilter nach https://dspguide.com/ch15/5.htm
    p = (filterbreite-1) / 2;
    q = p + 1;
    
    % erste Elemente
    for nn=1:1:q
        for kk=1:1:filterbreite
            y(nn) = y(nn) + x(kk);
        end
    end
    
    % Mittlere Elemente
    for ii=q+1:1:x_len-p
        y(ii) = y(ii-1) + x(ii+p) - x(ii-q);
    end
    
    y = y / filterbreite;
end

% rekursiver Mittelwertsfilter. Breite muss ungerade sein. Kausal, dadurch
% langsamer um die Breite des Filters
function [y] = mittelwert_rekursiv_kausal(x, mw_breite)
    % wenn mw_breite gerade, dann um 1 erhoehen.
    filterbreite = mw_breite + not(mod(mw_breite,2));
    x_len=length(x);
    y = zeros(x_len, 1);
    
    % rekursiver Mittelwertsfilter nach https://dspguide.com/ch15/5.htm
    p = (filterbreite-1) / 2;
    q = p + 1;
    
    % erste Elemente
    for nn=1:1:q+p
        for kk=1:1:filterbreite
            y(nn) = y(nn) + x(kk);
        end
    end
    
    % Mittlere Elemente
    for ii=q+p+1:1:x_len-p
        y(ii) = y(ii-1) + x(ii) - x(ii-q-p);
    end
    
    y = y / filterbreite;
end

% eigener Versuch 0, einen schnelleren Mittelwertfilter zu designen.
function [y] = mittelwert_benedikt_quad(x,mw_breite)
    x_len = length(x);
    y=zeros(x_len,1);
    
    % quadratische Gewichtung. f(x) = x^2 / k, k = int(0,mw_breite) f(x)dx
    filter_vec = linspace(0,mw_breite, mw_breite);
    filter_vec = filter_vec.^2;
    stauchfaktor = mw_breite / sum(filter_vec, "all");
    filter_vec = filter_vec * stauchfaktor;
    
    for ii=mw_breite:x_len
        for kk=0:mw_breite-1
            y(ii,1) = y(ii,1) + x(ii-kk,1) * filter_vec(kk+1);
        end
    end
    
    y = y / mw_breite;
end

% eigener Versuch 1, einen schnelleren Mittelwertfilter zu designen.
function [y] = mittelwert_benedikt_arctan(x,mw_breite)
    x_len = length(x);
    y=zeros(x_len,1);
    
    % arctan - Gewichtung.
    filter_vec = linspace(0,mw_breite, mw_breite);
    
    a_arctan = -0.1;
    b_arctan = mw_breite / 4 * 3;
    c_arctan = mw_breite / 2;
    
    filter_vec = atan(a_arctan*(filter_vec - b_arctan) * mw_breite / 3.14 + c_arctan);
    stauchfaktor = mw_breite / sum(filter_vec, "all");
    filter_vec = filter_vec * stauchfaktor;
    
    %filter_vec = ones(mw_breite,1);
    
    for ii=mw_breite:x_len
        for kk=0:mw_breite-1
            y(ii,1) = y(ii,1) + x(ii-kk,1) * filter_vec(kk+1);
        end
    end
    
    y = y / mw_breite;
end

% Ableitung diskret
function [y] = ableitung_diskret(x, f_sample)
    vec_len = length(x);
    y = zeros(vec_len, 1);
    
    y(1)= x(1);
    for ii=2:1:vec_len
        y(ii) = (x(ii) - x(ii-1));
    end
    
    y = y * f_sample;
end

% berechnet die min + max Peaks im Puffer der Breite "mw_breite" und gibt den Wert als Mittelwert zwischen den beiden aus.
function [y] = peak_mittelwert_filter(x, mw_breite)
    x_len = length(x);
    y = zeros(x_len,1);
    
    for ii=mw_breite+1:1:x_len
        min_val = min(x(ii-mw_breite:ii));
        max_val = max(x(ii-mw_breite:ii));
        y(ii) = (min_val + max_val) / 2;
        %y(ii) = sqrt(min_val^2 + max_val^2);
    end
end

% Funktion, die die Standardabweichung uber den Filterbereich berechnet
function [y] = moving_stdev(x, filterbreite)
    x_len = length(x);
    y = zeros(x_len,1);
    
    for ii=filterbreite+1:1:x_len
        y(ii) = std(x(ii-filterbreite:ii));
    end
end

% Funktion, die Koeffizienten mit der Standardabweichung gewichtet. Je groesser die stdev, desto unwichtiger der Punkt.
function [y] = stdev_filter(x, filterbreite)
    x_stdev = moving_stdev(x, filterbreite);
    x_norm = x_stdev / max(x_stdev);
    
    y = x./x_norm;
end

function [y] = FIR(x)
    vec_len = length(x);
    y=zeros(vec_len,1);
    
    % Stop 1Hz
    %filter_vec = transpose([0.000412456641819414,5.09094875612174e-05,5.40373105166814e-05,5.72512166909346e-05,6.06052616331063e-05,6.40495402466277e-05,6.76409893331147e-05,7.13333746568438e-05,7.51804132246538e-05,7.91334806420115e-05,8.32482184670746e-05,8.74739640769304e-05,9.18632926916733e-05,9.63622473740042e-05,0.000101026853722463,0.000105800527188963,0.000110740977945828,0.000115791734005400,0.000121019506020420,0.000126366224387503,0.000131902359006987,0.000137566332006323,0.000143436288291931,0.000149441151437332,0.000155657431483112,0.000161997512589425,0.000168541938008930,0.000175188426018469,0.000182031454543571,0.000188966256253148,0.000196128117723483,0.000203412927573058,0.000210995723335707,0.000218727529640003,0.000226778378282288,0.000234847395653230,0.000243127162055303,0.000251263812764202,0.000260178424379581,0.000269229363164003,0.000277958719006498,0.000287409118311872,0.000296741692079031,0.000306462705023980,0.000316244199471962,0.000326342312096204,0.000336554453701458,0.000347055040033995,0.000357694044985013,0.000368607918622732,0.000379674783961995,0.000391002972615661,0.000402491469291058,0.000414235934257761,0.000426149299860942,0.000438316016845354,0.000450663095885943,0.000463270844186281,0.000476070793699434,0.000489129965393354,0.000502380443645255,0.000515883269403680,0.000529573504486851,0.000543510947289870,0.000557641152495159,0.000572030968478461,0.000586628319945524,0.000601490144911251,0.000616550077986780,0.000631853773568400,0.000647334209759525,0.000663065669171136,0.000679012990842409,0.000695260128950576,0.000711695940500654,0.000728340894509701,0.000745124638791634,0.000762350488254539,0.000779612064696364,0.000797165186292604,0.000814948198862697,0.000832936693947956,0.000851161915640149,0.000869603366819820,0.000888274063054352,0.000907166617088849,0.000926278228788151,0.000945609879656978,0.000965156771700651,0.000984922259604424,0.00100489775175521,0.00102509333338901,0.00104550058940937,0.00106612652041757,0.00108695560548901,0.00110799631664586,0.00112923503119097,0.00115068218870258,0.00117232418958633,0.00119417570814012,0.00121622342670229,0.00123847707577437,0.00126091429695385,0.00128354301097172,0.00130634501741622,0.00132933864086257,0.00135251313383930,0.00137588656440665,0.00139943003828816,0.00142314519362568,0.00144700981825475,0.00147106211165346,0.00149529178258087,0.00151969105341785,0.00154419749520068,0.00156888830211753,0.00159376605497691,0.00161872197332068,0.00164388068913046,0.00166914818591354,0.00169457118064511,0.00172011495657189,0.00174579317056981,0.00177159326867047,0.00179751722033121,0.00182355547037399,0.00184971047631742,0.00187597553410410,0.00190234446927609,0.00192880921414691,0.00195536720710908,0.00198201563024615,0.00200875005944614,0.00203556676830786,0.00206246235343565,0.00208943263100676,0.00211646892608767,0.00214356382114347,0.00217071237971454,0.00219791320650852,0.00222516380847498,0.00225245904013980,0.00227979075396442,0.00230714790777170,0.00233452578821978,0.00236192379675266,0.00238934628332699,0.00241677947892235,0.00244421063807344,0.00247162613476938,0.00249904602029450,0.00252646093126215,0.00255385407272568,0.00258119677098706,0.00260854627512835,0.00263584171850609,0.00266308475570457,0.00269029813577334,0.00271743955423901,0.00274453968920722,0.00277155894914405,0.00279852205245533,0.00282539879800453,0.00285220014518942,0.00287890011051511,0.00290551460731696,0.00293202165443471,0.00295842988534148,0.00298471688424853,0.00301089326852613,0.00303693792747375,0.00306285681477554,0.00308862937530572,0.00311426613791463,0.00313975004664232,0.00316508762038465,0.00319025702639542,0.00321526335110138,0.00324008803236380,0.00326474067350737,0.00328920572335411,0.00331348925048369,0.00333756704893059,0.00336144299306528,0.00338510353060634,0.00340856362375256,0.00343180131227383,0.00345481577628885,0.00347758613865485,0.00350013694530045,0.00352243993116898,0.00354449453144812,0.00356628173418147,0.00358784080924889,0.00360910720818033,0.00363012445620893,0.00365085354286304,0.00367131423399493,0.00369148003894996,0.00371135958545415,0.00373094229802465,0.00375023256648560,0.00376921100187118,0.00378788324497628,0.00380623705396078,0.00382427726433336,0.00384198656965275,0.00385937120978612,0.00387641952623432,0.00389313782980553,0.00390950865855935,0.00392553626300980,0.00394120729476837,0.00395653075062142,0.00397149317048344,0.00398610044737457,0.00400033492913023,0.00401420153218657,0.00402768676373418,0.00404080312554857,0.00405353689054928,0.00406589220957858,0.00407784961912411,0.00408942209422931,0.00410059826989177,0.00411138595225602,0.00412176135839468,0.00413174128492454,0.00414131725408870,0.00415049056002664,0.00415924247445076,0.00416758687824202,0.00417552155089206,0.00418303066920940,0.00419012626963148,0.00419679940681326,0.00420305697546974,0.00420888275737068,0.00421428381533095,0.00421926102857069,0.00422381110349492,0.00422792943365985,0.00423161603305077,0.00423487449346801,0.00423770026076981,0.00424009122118938,0.00424204456775244,0.00424356622506052,0.00424465466666434,0.00424531088669107,0.00424552896170033,0.00424531088669107,0.00424465466666434,0.00424356622506052,0.00424204456775244,0.00424009122118938,0.00423770026076981,0.00423487449346801,0.00423161603305077,0.00422792943365985,0.00422381110349492,0.00421926102857069,0.00421428381533095,0.00420888275737068,0.00420305697546974,0.00419679940681326,0.00419012626963148,0.00418303066920940,0.00417552155089206,0.00416758687824202,0.00415924247445076,0.00415049056002664,0.00414131725408870,0.00413174128492454,0.00412176135839468,0.00411138595225602,0.00410059826989177,0.00408942209422931,0.00407784961912411,0.00406589220957858,0.00405353689054928,0.00404080312554857,0.00402768676373418,0.00401420153218657,0.00400033492913023,0.00398610044737457,0.00397149317048344,0.00395653075062142,0.00394120729476837,0.00392553626300980,0.00390950865855935,0.00389313782980553,0.00387641952623432,0.00385937120978612,0.00384198656965275,0.00382427726433336,0.00380623705396078,0.00378788324497628,0.00376921100187118,0.00375023256648560,0.00373094229802465,0.00371135958545415,0.00369148003894996,0.00367131423399493,0.00365085354286304,0.00363012445620893,0.00360910720818033,0.00358784080924889,0.00356628173418147,0.00354449453144812,0.00352243993116898,0.00350013694530045,0.00347758613865485,0.00345481577628885,0.00343180131227383,0.00340856362375256,0.00338510353060634,0.00336144299306528,0.00333756704893059,0.00331348925048369,0.00328920572335411,0.00326474067350737,0.00324008803236380,0.00321526335110138,0.00319025702639542,0.00316508762038465,0.00313975004664232,0.00311426613791463,0.00308862937530572,0.00306285681477554,0.00303693792747375,0.00301089326852613,0.00298471688424853,0.00295842988534148,0.00293202165443471,0.00290551460731696,0.00287890011051511,0.00285220014518942,0.00282539879800453,0.00279852205245533,0.00277155894914405,0.00274453968920722,0.00271743955423901,0.00269029813577334,0.00266308475570457,0.00263584171850609,0.00260854627512835,0.00258119677098706,0.00255385407272568,0.00252646093126215,0.00249904602029450,0.00247162613476938,0.00244421063807344,0.00241677947892235,0.00238934628332699,0.00236192379675266,0.00233452578821978,0.00230714790777170,0.00227979075396442,0.00225245904013980,0.00222516380847498,0.00219791320650852,0.00217071237971454,0.00214356382114347,0.00211646892608767,0.00208943263100676,0.00206246235343565,0.00203556676830786,0.00200875005944614,0.00198201563024615,0.00195536720710908,0.00192880921414691,0.00190234446927609,0.00187597553410410,0.00184971047631742,0.00182355547037399,0.00179751722033121,0.00177159326867047,0.00174579317056981,0.00172011495657189,0.00169457118064511,0.00166914818591354,0.00164388068913046,0.00161872197332068,0.00159376605497691,0.00156888830211753,0.00154419749520068,0.00151969105341785,0.00149529178258087,0.00147106211165346,0.00144700981825475,0.00142314519362568,0.00139943003828816,0.00137588656440665,0.00135251313383930,0.00132933864086257,0.00130634501741622,0.00128354301097172,0.00126091429695385,0.00123847707577437,0.00121622342670229,0.00119417570814012,0.00117232418958633,0.00115068218870258,0.00112923503119097,0.00110799631664586,0.00108695560548901,0.00106612652041757,0.00104550058940937,0.00102509333338901,0.00100489775175521,0.000984922259604424,0.000965156771700651,0.000945609879656978,0.000926278228788151,0.000907166617088849,0.000888274063054352,0.000869603366819820,0.000851161915640149,0.000832936693947956,0.000814948198862697,0.000797165186292604,0.000779612064696364,0.000762350488254539,0.000745124638791634,0.000728340894509701,0.000711695940500654,0.000695260128950576,0.000679012990842409,0.000663065669171136,0.000647334209759525,0.000631853773568400,0.000616550077986780,0.000601490144911251,0.000586628319945524,0.000572030968478461,0.000557641152495159,0.000543510947289870,0.000529573504486851,0.000515883269403680,0.000502380443645255,0.000489129965393354,0.000476070793699434,0.000463270844186281,0.000450663095885943,0.000438316016845354,0.000426149299860942,0.000414235934257761,0.000402491469291058,0.000391002972615661,0.000379674783961995,0.000368607918622732,0.000357694044985013,0.000347055040033995,0.000336554453701458,0.000326342312096204,0.000316244199471962,0.000306462705023980,0.000296741692079031,0.000287409118311872,0.000277958719006498,0.000269229363164003,0.000260178424379581,0.000251263812764202,0.000243127162055303,0.000234847395653230,0.000226778378282288,0.000218727529640003,0.000210995723335707,0.000203412927573058,0.000196128117723483,0.000188966256253148,0.000182031454543571,0.000175188426018469,0.000168541938008930,0.000161997512589425,0.000155657431483112,0.000149441151437332,0.000143436288291931,0.000137566332006323,0.000131902359006987,0.000126366224387503,0.000121019506020420,0.000115791734005400,0.000110740977945828,0.000105800527188963,0.000101026853722463,9.63622473740042e-05,9.18632926916733e-05,8.74739640769304e-05,8.32482184670746e-05,7.91334806420115e-05,7.51804132246538e-05,7.13333746568438e-05,6.76409893331147e-05,6.40495402466277e-05,6.06052616331063e-05,5.72512166909346e-05,5.40373105166814e-05,5.09094875612174e-05,0.000412456641819414]);
    %filter_vec = transpose([0.00979675954092965,0.0338489367691140,0.0747740506147740,0.124202621867892,0.165429189844296,0.181546863613973,0.165429189844296,0.124202621867892,0.0747740506147740,0.0338489367691140,0.00979675954092965]);
    
    % FIR, Equiripple, DF20, F_s 100Hz, Fpass 5, Apass 1, Fstop 20, Astop
    % 200, 37th order
    %filter_vec = transpose([-1.31367914333445e-07,-1.25052120531083e-06,-6.29937147634608e-06,-2.15846043118762e-05,-5.39675825788786e-05,-9.51988477752890e-05,-8.15300769208984e-05,0.000178756720253494,0.00109991828387737,0.00339681759886638,0.00808104370531578,0.0163066955768710,0.0290376049009453,0.0465904014657846,0.0682022735404689,0.0918258349148108,0.114318748393234,0.132063716619290,0.141871468579411,0.141871468579411,0.132063716619290,0.114318748393234,0.0918258349148108,0.0682022735404689,0.0465904014657846,0.0290376049009453,0.0163066955768710,0.00808104370531578,0.00339681759886638,0.00109991828387737,0.000178756720253494,-8.15300769208984e-05,-9.51988477752890e-05,-5.39675825788786e-05,-2.15846043118762e-05,-6.29937147634608e-06,-1.25052120531083e-06,-1.31367914333445e-07]);
    
    
    %
    filter_vec = transpose([0.00277516610035745,0.00264798700255519,0.00381433603389304,0.00523051384575485,0.00689992264965136,0.00881835259634970,0.0109704524909869,0.0133268585535907,0.0158517627280251,0.0185011807429986,0.0212144950943307,0.0239333369571773,0.0265870816989550,0.0291070854458265,0.0314242546157388,0.0334730753897149,0.0351918377561769,0.0365292399442331,0.0374442682471306,0.0379084195828126,0.0379084195828126,0.0374442682471306,0.0365292399442331,0.0351918377561769,0.0334730753897149,0.0314242546157388,0.0291070854458265,0.0265870816989550,0.0239333369571773,0.0212144950943307,0.0185011807429986,0.0158517627280251,0.0133268585535907,0.0109704524909869,0.00881835259634970,0.00689992264965136,0.00523051384575485,0.00381433603389304,0.00264798700255519,0.00277516610035745]);
    
    order_max=length(filter_vec);
    for ii=order_max:vec_len
        for kk=0:order_max-1
            y(ii,1) = y(ii,1) + x(ii-kk,1) * filter_vec(kk+1,1);
        end
    end
end

% Funktion, die die maximale Aenderung zwischen zwei Werten auf begrenzt
function [y] = max_delta_filter(x, delta_wert)
    y = zeros(length(x),1);
    y(1) = x(1);
    %y = x;
    
    for ii=2:1:length(y)
        if (x(ii) > y(ii-1))
            y(ii) = y(ii-1) + delta_wert;
        else
            y(ii) = y(ii-1) - delta_wert;
        end
    end
end

% findet alle x, bei denen ein Wert <= dem Schwellwert vorliegt.
function [y] = neg_accel_finder(x, schwellwert)
    y = zeros(length(x),1);    
    y(x <= schwellwert,1) = 1;
end

% funktioniert wie max_delta, nur dass statt einem Delta der Wert high oder low wird.
function [y] = pwm_filter(x, schwellwert)
    y = zeros(length(x),1);
    y(x > schwellwert, 1) = 1 + schwellwert;
    y(x < schwellwert, 1) = -1 + schwellwert;
end

% extrapoliert Werte linear, um an Geschwindigkeit zu gewinnen
function [y] = extrapolate_lin(x,t, anzahl_werte_quelle_erzeugung)
    x_len = length(x);
    y = zeros(x_len,1);
    
    
    % wikipedia lineare regression: https://de.wikipedia.org/wiki/Lineare_Einfachregression
    for ii=1:x_len/anzahl_werte_quelle_erzeugung-1
        
        start = (1+(ii-1)*anzahl_werte_quelle_erzeugung);
        stop = (1+ii*anzahl_werte_quelle_erzeugung);
        
        mean_x = mean(x(start:stop));
        mean_t = mean(t(start:stop));

        steigung = sum((t(start:stop)-mean_t) .* (x(start:stop)-mean_x)) / sum((t(start:stop)-mean_t).^2);
        offset = mean_x - steigung * mean_t;

        y(start:stop) = steigung * t(start:stop) + offset;
    end
end

% Exponentieller MA Filter https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
function [y] = exp_moving_avg(x,alpha)
    y = zeros(length(x),1);
    y(1) = x(1);
    for ii=2:1:length(x)
        y(ii) = alpha * x(ii) + (1 - alpha) * y(ii-1);
    end
end

% Diskrete Integration
function [y] = integration_diskret(x, f_sample)
    y = zeros(length(x),1);
    y(1) = x(1) / f_sample;
    for ii=2:1:length(x)
        y(ii) = (y(ii-1) + x(ii) / f_sample);
    end
end

% Kumulativer Mittelwert, vielleicht fuer spaetere Offsetkompensation https://en.wikipedia.org/wiki/Moving_average#Cumulative_moving_average
function [y] = cumulative_avg(x)
    y = zeros(length(x),1);
    y(1) = x(1);
    for ii=2:1:length(x)
       y(ii) = y(ii-1) + (x(ii) - y(ii-1)) / (ii);
    end
end

% Funktion, die Steigung zwischen zwei Werten im Abstand delta berechnet ->
% fuer glattere Ableitung
function [y] = ableitung_mit_delta(x, delta, f_sample)
    y = zeros(length(x),1);
    
    for ii=delta+1:1:length(x)
        y(ii) = (x(ii) - x(ii - delta)) / delta;
    end
    
    y = y * f_sample;
end

% Bremsklassfikation
function [y] = wird_gebremst(wert0, schwellwert0, wert1, schwellwert1)
    y0 = zeros(length(wert0),1);
    y1 = y0;
    y = y0;
    
    y0(wert0 < schwellwert0) = 1;
    y1(wert1 < schwellwert1) = 1;
    
    y_logical = y0 & y1;
    y(y_logical ~= 0) = -2;
end

% Ein- und Ausschalthysterese. Uebernimmt high erst nach der Dauer t_on,
% low erst nach Dauer t_off
function [y] = hysterese(x, t_on, t_off, f_sample)
    x_len=length(x);
    y_h = zeros(x_len,1);
    t_run = 0;
    t_stop = 0;
    
    lo = 0;
    hi = -2;
    state = lo;
    
    for ii=1:1:x_len
        if (x(ii) == lo)
            t_run = 0;
            t_stop = t_stop + 1/f_sample;
        end
        
        if (x(ii) == hi)
            t_stop = 0;
            t_run = t_run + 1/f_sample;
        end
        
        if ((state == lo) && (t_run >= t_on))
            state = hi;
        end
        
        if ((state == hi) && (t_stop >= t_off))
            state = lo;
        end
        
        y_h(ii) = state;
    end
    
    y = y_h(1:x_len);
end

% Funktion, die die kumulative Standardabweichung bis von 1:x von x berechnet
function [y] = cumulative_stdev(x)
    y = zeros(length(x),1);
    
    for ii=1:1:length(x)
        y(ii) = std(x(1:ii));
    end
end

% Clippt die Werte zum +-Schwellwert, um Spitzen zu kürzen
function [y] = clip_vals(x, schwellwert)
    y = x;
    y(x > schwellwert) = schwellwert;
    y(x < -schwellwert) = - schwellwert;
end