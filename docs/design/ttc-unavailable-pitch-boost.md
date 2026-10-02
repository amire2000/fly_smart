# TTC-unavailable pitch boost

The default scenario uses a 5° pitch boost when the red target remains visible
but the TTC tracker has no valid observation. This keeps the drone moving on a
steady slant while bbox growth is unavailable or invalid, then returns to the
normal forward-speed PID as soon as TTC becomes valid.

The existing tracker minimum-growth rule remains the boundary for invalid TTC;
no second growth threshold or new guidance state is introduced. The fallback is
bounded by the normal maximum pitch plus the configured boost.
