// web/modules/panels/_util.js
//
// Tiny helpers shared by panel files. Kept private to panels/.

export function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
}

// Returns a fn(...args) that delays calling `fn` until `ms` of quiet.
// Used by sliders that send a command on `input`.
export function debounce(fn, ms) {
    let timer = null;
    return function (...args) {
        if (timer) clearTimeout(timer);
        timer = setTimeout(() => { timer = null; fn.apply(null, args); }, ms);
    };
}
