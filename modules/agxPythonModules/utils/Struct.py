from argparse import Namespace

class Struct(Namespace):
    """
    Yet another data class. Unlike self.__dict__.update this
    class support recursive dictionaries.

    Examples:
        data_dict = {
            'key1': 12.5,
            'another_key': {
                'value': 24.7,
                'foo': {
                    'bar': 3
                }
            }
        }

        data = Struct( data_dict )
        print( data.key1 )
        data.another_key.value = 10.0
        data.another_key.foo.bar = data.another_key.value
    """
    def __init__(self, data: dict):
        """
        Construct given dictionary.

        Arguments:
            data: dict - data dictionary
        """
        for k, v in data.items():
            if isinstance( v, (list, tuple) ):
                setattr( self, k, [Struct( inner ) if isinstance( inner, dict ) else inner for inner in v] )
            else:
                setattr( self, k, Struct( v ) if isinstance( v, dict ) else v )

    def __iter__(self):
        for attr, value in self.__dict__.items():
            if not attr.startswith( '__' ):
                yield attr, value