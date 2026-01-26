# Entry point of program
from GSCore.drivers.motive_client import print_motive_data
from GSCore.drivers.mock_motive_client import print_mock_data
def main():
    print_mock_data()
    
if __name__ == "__main__":
    main()