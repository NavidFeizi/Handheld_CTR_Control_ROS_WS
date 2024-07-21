#ifndef SYNCED_VECTOR_HPP
#define SYNCED_VECTOR_HPP

#include <cstddef>       // For size_t
#include <blaze/Blaze.h> // For blaze::StaticVector
#include <torch/torch.h> // For torch::Tensor

template <size_t N>
class SyncedVector
{
public:
    SyncedVector() : torchVec(torch::zeros({1, static_cast<long>(N)}, torch::kFloat32))
    {
        blazeVec.reset(); // Initialize blazeVec to zeros
    }

    void setBlazeVec(const blaze::StaticVector<float, N> &newVec)
    {
        blazeVec = newVec;
        updateTorchVec();
    }

    const blaze::StaticVector<float, N> &getBlazeVec() const
    {
        return blazeVec;
    }

    void setTorchVec(const torch::Tensor &newTensor)
    {
        torchVec = newTensor;
        updateBlazeVec();
    }

    const torch::Tensor &getTorchVec() const
    {
        return torchVec;
    }

private:
    blaze::StaticVector<float, N> blazeVec; // Blaze vector
    torch::Tensor torchVec;                 // Torch tensor

    void updateTorchVec()
    {
        auto acc = torchVec.accessor<float, 2>();
        for (size_t i = 0; i < N; ++i)
        {
            acc[0][i] = blazeVec[i];
        }
    }

    void updateBlazeVec()
    {
        auto acc = torchVec.accessor<float, 2>();
        for (size_t i = 0; i < N; ++i)
        {
            blazeVec[i] = acc[0][i];
        }
    }
};

#endif // SYNCED_VECTOR_HPP
