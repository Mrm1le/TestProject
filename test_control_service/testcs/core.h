#pragma once
class T
{
public:
    T(int in)
    {
        a = in;
    }
    int a;
    static T* get();
    int change(int in);
private:
    // static T *t;
    static T *core;
};


