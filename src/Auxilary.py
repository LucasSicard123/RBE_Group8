import math 

class Auxilary:

    

    def __init__(self):
        return
    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(value, max_value))
    @staticmethod
    def rangeCheck(value, min_Value, max_value):
        return value >= min_Value and value <= max_value
    
        
