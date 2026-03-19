import csv

with open('output/centerline_data.csv', 'r') as f:
    reader = csv.DictReader(f)
    rows = list(reader)

green_detected = [r for r in rows if r['Green_Box_Detected'] == 'True']
green_under_red_true = [r for r in green_detected if r['Green_Under_Red'] == 'True']
green_under_red_false = [r for r in green_detected if r['Green_Under_Red'] == 'False']

print('=== BATCH PROCESSING RESULTS ===')
print(f'Total images processed: {len(rows)}')
print(f'Green boxes detected: {len(green_detected)}')
print()
print('=== GREEN BOXES WITH RED PIERCING THROUGH ===')
for row in green_under_red_true:
    print(f"  {row['Image']} | Red piercing detected")
print()
print('=== GREEN BOXES WITH RED TO THE SIDE ===')
for row in green_under_red_false:
    print(f"  {row['Image']} | Red not piercing")
print()
print(f'Total debug images saved: {len(green_detected) * 3} (3 per detection)')
