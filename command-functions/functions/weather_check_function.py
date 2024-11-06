# Owen Bartlett
# This function uses the open meteo api to get recent data based on a given coordinate
# It is currently set to GET only the data that is relevant to flying, but there are 
# additional parameters that can be found here: https://open-meteo.com/en/docs

import openmeteo_requests
import requests_cache
import pandas as pd
from retry_requests import retry

def check_weather(lat_takeoff, lon_takeoff):
        # Setup the Open-Meteo API client with cache and retry on error
    cache_session = requests_cache.CachedSession('.cache', expire_after = 3600)
    retry_session = retry(cache_session, retries = 5, backoff_factor = 0.2)
    openmeteo = openmeteo_requests.Client(session = retry_session)

    # Make sure all required weather variables are listed here
    # The order of variables in hourly or daily is important to assign them correctly below
    url = "https://api.open-meteo.com/v1/forecast"
    params = {
        "latitude": lat_takeoff,
        "longitude": lon_takeoff,
        "forecast_days": 1,
        "hourly": {"visibility", "wind_speed_10m", "precipitation",}
    }
    responses = openmeteo.weather_api(url, params=params)

    # Process first location. Add a for-loop for multiple locations or weather models
    response = responses[0]
    print(f"Coordinates {response.Latitude()}°N {response.Longitude()}°E")
    print(f"Elevation {response.Elevation()} m asl")


    # Process hourly data. The order of variables needs to be the same as requested.
    hourly = response.Hourly()
    hourly_visibility = hourly.Variables(0).ValuesAsNumpy()
    hourly_wind_speed_10 = hourly.Variables(1).ValuesAsNumpy()
    hourly_precipitation = hourly.Variables(2).ValuesAsNumpy()
    
    # order data to be printed
    hourly_data = {}
    hourly_data["wind speed 10m"] = hourly_wind_speed_10
    hourly_data["precipitation"] = hourly_precipitation
    hourly_data["visibility"] = hourly_visibility

    # display data
    hourly_dataframe = pd.DataFrame(data = hourly_data)
    print(hourly_dataframe)

    # calculate average (this is incredibly innacurate but should demonstrate the concept for now)
    avg_wind_speed = hourly_wind_speed_10.mean()
    avg_precipitation = hourly_precipitation.mean()
    avg_visibility = hourly_visibility.mean()
    
    # arbitrary parameters that we want to compare to
    MAX_WIND_SPEED = 15
    MAX_PRECIPITATION = 0.1
    MIN_VISIBILITY = 500

    # compare to parameters and fail if criteria is met
    if avg_wind_speed > MAX_WIND_SPEED:
        print("Wind speed is too high for safe flight. Shutting down UAV.")
        return False
    if avg_precipitation > MAX_PRECIPITATION:
        print("Precipitation detected. Shutting down UAV.")
        return False
    if avg_visibility < MIN_VISIBILITY:
        print("Visibility is too low for safe flight. Shutting down UAV.")
        return False
    
    return True

check_weather(39.32, -76.62)