# Functions which are used by multiple modules
    
    
# Bound values between saturation limits
def sat_values(value, sat_lower, sat_upper):
    if value >= sat_upper:
        return sat_upper
    elif value <= sat_lower:
        return sat_lower
    else:
        return value