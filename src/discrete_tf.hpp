#pragma once

template <typename T>
class DiscreteTf
{
private:
    /* data */
public:
    virtual T Step(T)         = 0;
    virtual void ResetState() = 0;
};
