#pragma once

namespace informed_sampler_guard
{
    // When true, allocators should avoid constructing InformedStateSamplers
    // and instead return a simple fallback sampler to prevent recursion.
    inline thread_local bool constructing = false;
}
