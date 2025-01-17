

THROTTLE_SAMPLE = [54.761904761904766, 62.698412698412696, 70.63492063492063, 78.57142857142858, 86.5079365079365];
PWM_SAMPLE = THROTTLE_SAMPLE / 100 * 1000 + 1000;

RPM_SAMPLE = [5610, 6180, 6690, 7200, 7680];
%RPM_SAMPLE = [6600, 8100, 9240, 9870, 10260, 10500, 12600]; % Valori di RPM
MAX_PWM = 2000;

% Calcolo dei coefficienti del polinomio di grado 1
coeffs_1 = polyfit(THROTTLE_SAMPLE, RPM_SAMPLE, 1);
a_1 = coeffs_1(1);
b_1 = coeffs_1(2);

% Stampa i coefficienti del polinomio
fprintf('Coefficiente a (x^1): %.20f\n', a_1);
fprintf('Coefficiente b (x^0): %.20f\n', b_1);

% Calcolo dei coefficienti del polinomio di grado 2
coeffs = polyfit(RPM_SAMPLE, PWM_SAMPLE, 3);

% Coefficienti del polinomio
a = coeffs(1); % Coefficiente di x^3
b = coeffs(2); % Coefficiente di x^2
c = coeffs(3); % Coefficiente di x^1
d = coeffs(4); % Coefficiente costante

% Definizione della funzione lambda basata sui coefficienti calcolati
rpm_model = @(rpm) a*rpm.^3 + b*rpm.^2 + c*rpm + d;
pwm_model = @(throttle) (a_1*throttle + b_1);

% Esempio di utilizzo
rpm_values = linspace(min(RPM_SAMPLE), 12600, 100); % Gamma di valori RPM
pwm_values = rpm_model(rpm_values); % Calcola le PWM corrispondenti

% Visualizzazione dei risultati
throttles = linspace(0, 100, 10);
plot(throttles, pwm_model(throttles), 'r-', 'LineWidth', 2);
hold on;
scatter(THROTTLE_SAMPLE, RPM_SAMPLE);
grid on;
xlabel('THROTTLE');
ylabel('RPM');
legend('Linear regression', 'Dati misurati');

figure();

plot(rpm_values, pwm_values, 'r-', 'LineWidth', 2);
hold on;
scatter(RPM_SAMPLE, PWM_SAMPLE, 'bo', 'LineWidth', 1.5);
title('Relazione RPM vs PWM');
xlabel('RPM');
ylabel('PWM');
legend('Modello polinomiale', 'Dati misurati');
grid on;
hold off;

% Stampa i coefficienti del polinomio
fprintf('Coefficiente a (x^3): %.20f\n', a);
fprintf('Coefficiente b (x^2): %.20f\n', b);
fprintf('Coefficiente c (x^1): %.20f\n', c);
fprintf('Coefficiente d (x^0): %.20f\n', d);
