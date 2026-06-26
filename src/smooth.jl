# Small smooth primitives used to avoid non-differentiable hard clamps.

@inline function _smoothabs(x::Real, eps::Real)
    xx, ee = promote(x, eps)
    return sqrt(xx * xx + ee * ee)
end

@inline function _smoothmax(a::Real, b::Real, eps::Real)
    aa, bb, ee = promote(a, b, eps)
    two = one(aa) + one(aa)
    return (aa + bb + sqrt((aa - bb) * (aa - bb) + ee * ee)) / two
end

@inline function _smoothmin(a::Real, b::Real, eps::Real)
    aa, bb, ee = promote(a, b, eps)
    two = one(aa) + one(aa)
    return (aa + bb - sqrt((aa - bb) * (aa - bb) + ee * ee)) / two
end

@inline function _smoothclamp(x::Real, lo::Real, hi::Real, eps::Real)
    return _smoothmin(_smoothmax(x, lo, eps), hi, eps)
end
