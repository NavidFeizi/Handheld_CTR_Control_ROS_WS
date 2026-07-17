import numpy as np
import os


def sample_collocation(
    n,
    beta1_bound,
    beta2_bound,
    alpha1_bound,
    alpha2_bound,
    seed=None,
):
    """
    Uniform samples of (beta1, beta2, alpha1, alpha2) with:
      alpha1 in [alpha1_bound[0], alpha1_bound[1]]
      alpha2 in [alpha2_bound[0], alpha2_bound[1]]
      beta1 in [beta1_bound[0], beta1_bound[1]]
      beta2 in [beta2_bound[0], beta2_bound[1]]
      with modifications: beta2 += beta3, beta1 += beta2
    Returns: (n, 4) array [beta1, beta2, alpha1, alpha2]
    """
    rng = np.random.default_rng(seed)

    def biased_uniform(low, high, size, bias=2.0):
        """
        Sample from [low, high] with linearly increasing probability toward high.
        bias > 1 means more weight on higher values.
        """
        # Uniform [0,1)
        r = rng.random(size)
        # Apply power transformation (bias > 1 skews toward high)
        skewed = r ** (1 / bias)
        # Scale to [low, high]
        return low + (high - low) * skewed

    # Alphas: easy
    alpha1 = biased_uniform(alpha1_bound[0], alpha1_bound[1], n, bias=1.0)
    alpha2 = biased_uniform(alpha2_bound[0], alpha2_bound[1], n, bias=1.0)
    beta1  = biased_uniform(beta1_bound[0],  beta1_bound[1],  n, bias=1.5)
    beta2  = biased_uniform(beta2_bound[0],  beta2_bound[1],  n, bias=1.5)
    beta1 += beta2
    alpha1 += alpha2

    ### Stack
    tau = np.zeros((n, 4))
    tau[:, 0] = alpha1
    tau[:, 1] = beta1
    tau[:, 2] = alpha2
    tau[:, 3] = beta2

    return tau


# Example
if __name__ == "__main__":

    beta1_bound = [-0.084, -0.030]
    beta2_bound = [-0.072, -0.034]
    alpha1_bound = [-np.pi, np.pi]
    alpha2_bound = [-np.pi * 1.5, np.pi * 1.5]

    samples = sample_collocation(
        1000, beta1_bound, beta2_bound, alpha1_bound, alpha2_bound, seed=65
    )

    os.makedirs("Input_Files", exist_ok=True)
    np.savetxt("Input_Files/RandomSamples.csv", samples, delimiter=",", header="alpha1,beta1,alpha2,beta2", fmt='%.6f')

    print(samples.shape, "\n", samples[:5])
