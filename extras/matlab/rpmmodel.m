PWM_SAMPLE = [1200, 1250, 1300, 1350, 1400, 1450, 1500]; % Valori di PWM
RPM_SAMPLE = [5700, 6240, 6600, 8100, 9240, 9870, 10260]; % Valori di RPM
MAX_PWM = 2000;

% Calcolo dei coefficienti del polinomio di grado 2
coeffs = polyfit(RPM_SAMPLE, PWM_SAMPLE, 3);

% Coefficienti del polinomio
a = coeffs(1); % Coefficiente di x^3
b = coeffs(2); % Coefficiente di x^2
c = coeffs(3); % Coefficiente di x^1
d = coeffs(4); % Coefficiente costante

% Definizione della funzione lambda basata sui coefficienti calcolati
rpm_model = @(rpm) a*rpm.^3 + b*rpm.^2 + c*rpm + d;

% Esempio di utilizzo
rpm_values = linspace(min(RPM_SAMPLE), 12600, 100); % Gamma di valori RPM
pwm_values = rpm_model(rpm_values); % Calcola le PWM corrispondenti

% Visualizzazione dei risultati
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
