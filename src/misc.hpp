/*
 * misc.hpp
 *
 *  Created on: Jun 6, 2018
 *      Author: zak
 */

#ifndef SRC_MISC_HPP_
#define SRC_MISC_HPP_

class Quaterion {
private:
    float x;
    float y;
    float z;
    float w;

public:
    Quaterion(float w, float x, float y, float z) :
        x(x),
        y(y),
        z(z),
        w(w)
    { }

    Quaterion() :
        w(1.0f),
        x(0.0f),
        y(0.0f),
        z(0.0f) { }

    float get_x() { return x; }
    float get_y() { return y; }
    float get_z() { return z; }

    static Quaterion FromEulerAngles(float x, float y, float z)
    {
        float phi = x * 0.5f;
        float theta = y * 0.5f;
        float psi = z * 0.5f;

        float cphi = cosf(phi);
        float sphi = sinf(phi);
        float ctheta = cosf(theta);
        float stheta = sinf(theta);
        float cpsi = cosf(psi);
        float spsi = sinf(psi);

        float ww = cphi * ctheta * cpsi + sphi * stheta * spsi;
        float xx = sphi * ctheta * cpsi - cphi * stheta * spsi;
        float yy = cphi * stheta * cpsi + sphi * ctheta * spsi;
        float zz = cphi * ctheta * spsi - sphi * stheta * cpsi;

        Quaterion q(ww, xx, yy, zz);
        return q;
    }

    // 0 - w
    // 1 - x
    // 2 - y
    // 3 - z

    void GetEulerAngles(float &xx, float &yy, float &zz) {
        float R13, R11, R12, R23, R33;
        float q2s = y * y;

        R11 = 1.0f - 2.0f * (q2s + z * z);
        R12 = 2.0f * (w * z + x * y);
        R13 = 2.0f * (w * y - x * z);
        R23 = 2.0f * (w * x + y * z);
        R33 = 1.0f - 2.0f * (q2s + x * x);

        yy = asinf(R13);   // roll always between -pi/2 to pi/2
        zz = atan2f(R12, R11);
        xx = atan2f(R23, R33);
    }

    void normalize() {
        float l = sqrt(w * w + x * x + y * y + z * z);
        w /= l;
        x /= l;
        y /= l;
        z /= l;
    }

    Quaterion operator * (const Quaterion& q)
    {
        return Quaterion(
            w*q.w - x * q.x - y * q.y - z * q.z,
            w*q.x + x * q.w + y * q.z - z * q.y,
            w*q.y + y * q.w + z * q.x - x * q.z,
            w*q.z + z * q.w + x * q.y - y * q.x);
    }

    //Quaterion operator / (Quaterion& q)
    //{
    //  return ((*this) * (q.inverse()));
    //}

    Quaterion inverse() {
        Quaterion q = this->conjugate();
        q.normalize();
        return q;
    }

    Quaterion conjugate() {
        return Quaterion(w, -x, -y, -z);
    }
};



#endif /* SRC_MISC_HPP_ */
