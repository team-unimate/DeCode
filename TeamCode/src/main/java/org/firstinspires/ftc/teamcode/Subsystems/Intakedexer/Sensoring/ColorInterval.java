package org.firstinspires.ftc.teamcode.Subsystems.Intakedexer.Sensoring;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorInterval {
    double rMin, rMax;
    double gMin, gMax;
    double bMin, bMax;
    double tolerance; // margem de erro (0.0 a 1.0)

    public ColorInterval(double rMin, double rMax, double gMin, double gMax, double bMin, double bMax) {
        this(rMin, rMax, gMin, gMax, bMin, bMax, 0.15); // 15% de tolerância padrão
    }

    public ColorInterval(double rMin, double rMax, double gMin, double gMax, double bMin, double bMax, double tolerance) {
        this.rMin = rMin;
        this.rMax = rMax;
        this.gMin = gMin;
        this.gMax = gMax;
        this.bMin = bMin;
        this.bMax = bMax;
        this.tolerance = tolerance;
    }

    public boolean IsCoincident(NormalizedRGBA rgba) {
        // calcula centro de cada faixa
        double rCenter = (rMin + rMax) / 2.0;
        double gCenter = (gMin + gMax) / 2.0;
        double bCenter = (bMin + bMax) / 2.0;

        // calcula tolerância absoluta com base na largura do intervalo
        double rTol = (rMax - rMin) / 2.0 * (1 + tolerance);
        double gTol = (gMax - gMin) / 2.0 * (1 + tolerance);
        double bTol = (bMax - bMin) / 2.0 * (1 + tolerance);

        // verifica se o valor está dentro da faixa expandida
        return (Math.abs(rgba.red - rCenter) <= rTol &&
                Math.abs(rgba.green - gCenter) <= gTol &&
                Math.abs(rgba.blue - bCenter) <= bTol);
    }
}
